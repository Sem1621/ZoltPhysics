# Zig bug report: concurrent `processSpawnWindows` calls inherit each other's pipe handles

> **Status: already fixed in Zig 0.17.0. Do not file this as a new bug.**
>
> This report was written against Zig 0.16.0, the version ZoltPhysics uses. Before writing it, newer Zig was
> checked for the same race:
>
> - **Zig 0.17.0** (released in October 2026; on PyPI since 2026-10-08) serializes Windows spawns. In 0.17.0, `lib/std/Io/Threaded.zig`
>   adds the field `process_spawn_mutex: if (is_windows) Io.Mutex else void` (line 88).
>   - `processSpawnWindows` locks it (lines 15999-16000, unlocked by `defer`) before it creates the pipes (16014-16041).
>   - The child ends are closed in the `return` expression (16265-16296), which runs before the deferred unlock.
>   - This is the lock alternative described under "Proposed fix" below.
> - **Zig master was not checked.** Master is on Codeberg, which the environment this report was written in
>   cannot reach, and the GitHub mirror stops at the move to Codeberg. The check above used the 0.17.0 release
>   tarball (PyPI package `ziglang==0.17.0`).
> - **What 0.17.0 leaves open:** the mutex belongs to one `Io.Threaded` instance, not the whole process. Also,
>   `CreateProcessW` still runs with `bInheritHandles = TRUE` and no handle list. See
>   [Possible follow-up](#possible-follow-up-for-0170-and-later). That part could still be raised upstream, for
>   example on Codeberg ziglang/zig#30727 ("`std.process.Child` and handles").
> - **For ZoltPhysics:** after the move to Zig 0.17, the Windows `--test-timeout 10m` workaround should no longer
>   be needed. Confirm this before removing it, because 0.17.0 reworked the test protocol code in
>   `lib/std/Build/Step/Run.zig`: the "failed to respond" message no longer appears in that file.

---

## Summary

On Windows, `std.Io.Threaded.processSpawnWindows`:
- creates the child ends of the stdio pipes as inheritable handles;
- then calls `CreateProcessW` with `bInheritHandles = TRUE`;
- takes no lock around these two steps and passes no `PROC_THREAD_ATTRIBUTE_HANDLE_LIST`.

When two threads spawn at the same time, child B can inherit the pipe ends that thread A just created for child
A. Child A's stdout then reaches EOF only after child B has exited too.

In `zig build test` with several test binaries, a short test binary then fails with
`error: test runner failed to respond for 1m...` while a long-running sibling is still running. POSIX does not
have the problem: there the pipes are created with `O_CLOEXEC`.

- Zig version: 0.16.0
- OS: Windows (GitHub Actions `windows-latest`, Debug builds)

## Details

All line numbers refer to Zig 0.16.0, `lib/std/Io/Threaded.zig`, function `processSpawnWindows` (line 15457).

1. **The child ends are created inheritable** (lines 15472-15499). This covers the client (child) ends of the
   stdin, stdout and stderr pipes and of the progress pipe:

   ```zig
   const stdout_pipe = if (options.stdout == .pipe) try t.windowsCreatePipe(.{
       .server = .{ .attributes = .{ .INHERIT = false }, .mode = .{ .IO = .ASYNCHRONOUS } },
       .client = .{ .attributes = .{ .INHERIT = true }, .mode = .{ .IO = .SYNCHRONOUS_NONALERT } },
       .inbound = true,
   }) else undefined;
   ```

   Handles for `.file` stdio are opened inheritable as well (`.bInheritHandle = .TRUE`, lines 15512-15516 and
   the two blocks after it).

2. **The child is started with inheritance on.** `processSpawnWindows` fills a plain `STARTUPINFOW` (line
   15501), and `windowsCreateProcess` passes `bInheritHandles = TRUE` (lines 16236-16241):

   ```zig
   if (windows.kernel32.CreateProcessW(
       app_name,
       cmd_line,
       null,
       null,
       .TRUE,
   ```

   There is no `STARTUPINFOEXW`, no `EXTENDED_STARTUPINFO_PRESENT` and no `PROC_THREAD_ATTRIBUTE_HANDLE_LIST`.
   Nothing in `lib/std` calls `InitializeProcThreadAttributeList` or `UpdateProcThreadAttribute`.

3. **The parent closes its copies of the child ends late:** only after `CreateProcessW` has returned (lines
   15745-15785).

4. **No lock covers steps 1 to 3.**

So there is a window on thread A: it opens when `windowsCreatePipe` creates A's child ends, and it closes when
thread A calls `CloseHandle` on them. Any `CreateProcessW(bInheritHandles = TRUE)` on another thread during that
window copies A's child ends into its own child, B. The reason is that Windows passes every inheritable handle of
the process to the new process, not only the ones named in `STARTUPINFOW`.

The write end of A's stdout pipe then stays open until both A and B have exited. The parent therefore sees EOF
on A's stdout only when B has exited as well.

The POSIX path of the same file avoids this race on purpose. `spawnPosix` creates its pipes with
`pipe2(.{ .CLOEXEC = true })` (lines 14872-14904), and the comment there describes exactly this case:

> If another thread is racing to spawn a different child process, we don't want it to inherit these FDs in any
> scenario; that would mean that, for instance, calls to `poll` from the parent would not report the child's
> stdout as closing when expected, since the other child may retain a reference to the write end of the pipe.

The Windows path has no equivalent protection.

## How it shows up in `zig build test`

The build runner spawns test binaries (and the compiler) from parallel threads. In `lib/std/Build/Step/Run.zig`:

- After the last test, `requestNextTest` sends `.exit` to the test runner (lines 2543-2559).
- `waitZigTest` then waits for more output or for EOF on the child's stdout.
  - While no test is active it waits up to `response_timeout` (line 1859).
  - `response_timeout` is `@max(unit_test_timeout_ns orelse 0, 60 * std.time.ns_per_s)` (lines 1844-1847).
- If neither arrives in time, the step fails with `test runner failed to respond for {f}` (line 1791).

Suppose a test binary's stdout write end was inherited by a long-running sibling. The binary exits after
`.exit`, but its stdout does not reach EOF. After 60 s the build runner reports it as unresponsive.

The failure therefore needs a short test binary next to a sibling that keeps running for more than 60 s after
the short one is done. It is also intermittent, because the two spawns must overlap.

## Evidence

The evidence comes from the [Sem1621/ZoltPhysics](https://github.com/Sem1621/ZoltPhysics) CI: `windows-latest`,
Debug, Zig 0.16.0. Its `zig build test` runs four test binaries in parallel.

- **[Run 23](https://github.com/Sem1621/ZoltPhysics/actions/runs/39805113627)**, job
  [Test windows-latest Debug double_precision=true](https://github.com/Sem1621/ZoltPhysics/actions/runs/39805113627/job/119916151073).
  The binary `zolt-unit-tests-determinism-log` takes about 24 ms. It failed while its sibling `zolt-unit-tests`
  ran for 3 min:

  ```
  +- run test zolt-unit-tests-determinism-log failure
  error: test runner failed to respond for 1m10.043ms
  failed command: ".\\.zig-cache\\o\\7c40628c88d4eca64f897751a18363a8\\zolt-unit-tests-determinism-log.exe" "--cache-dir=.\\.zig-cache" --seed=0x3ad98531 --listen=-
  ...
  +- run test zolt-unit-tests 774 pass, 16 skip (790 total) 3m MaxRSS:62M
  ```

- **[Run 28](https://github.com/Sem1621/ZoltPhysics/actions/runs/40120758491)**, job
  [Test windows-latest Debug double_precision=false](https://github.com/Sem1621/ZoltPhysics/actions/runs/40120758491/job/120951570376).
  `zolt-tests` takes about 25 s in this configuration. It failed the same way next to `zolt-unit-tests`, which
  ran for over a minute ("1m" in the build summary). The same log shows
  `zolt-unit-tests-determinism-log 3 pass (3 total) 24ms`.

- **[Run 29, attempt 1](https://github.com/Sem1621/ZoltPhysics/actions/runs/40121846646/job/120954928839).**
  `zolt-unit-tests-determinism-log` failed again, while `zolt-unit-tests` ran for 3 min. The re-run passed.

In every failure, a short binary had finished while a long one was still running. The Linux and macOS test rows
of the same runs all passed.

**Workaround:** `--test-timeout 10m` on the Windows rows.
- It raises `response_timeout` to 10 min, so the build runner waits until the sibling has exited. It also limits
  every test to 10 min.
- It is in the `Test` step of
  [`.github/workflows/zolt.yml`](https://github.com/Sem1621/ZoltPhysics/blob/4eca03ce06d85cb1dd9ae7b96357e5b2e8a74820/.github/workflows/zolt.yml#L127-L137),
  whose comment explains the mechanism ([Sem1621/ZoltPhysics#15](https://github.com/Sem1621/ZoltPhysics/pull/15)).

## Reproduction

This reproduction follows from the mechanism above; it has not been run on Windows. The three files compile
with Zig 0.16.0 for `x86_64-windows`. On Linux, `zig build test` with these files passed in all three tries,
in about 90 s each.

`build.zig`:

```zig
const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const test_step = b.step("test", "Run the tests");
    for ([_][]const u8{ "slow", "empty" }) |name| {
        const tests = b.addTest(.{
            .name = name,
            .root_module = b.createModule(.{
                .root_source_file = b.path(b.fmt("{s}.zig", .{name})),
                .target = target,
            }),
        });
        const run = b.addRunArtifact(tests);
        // Run the tests on every `zig build test`, also when nothing changed.
        run.has_side_effects = true;
        test_step.dependOn(&run.step);
    }
}
```

`slow.zig`:

```zig
const std = @import("std");

test "sleep 90 s" {
    try std.testing.io.sleep(.fromSeconds(90), .awake);
}
```

`empty.zig`:

```zig
// No tests: the test runner exits as soon as the build runner sends `.exit`.
```

Run `zig build test --summary all` several times on Windows.

- **Expected with the bug:** in some runs the `empty` step fails with
  `error: test runner failed to respond for 1m...` while `slow` is still sleeping.
- **Expected without the bug:** every run passes and takes about 90 s.

The two spawns must overlap, so the failure is intermittent. Two things raise the hit rate:
- `has_side_effects = true` makes the tests run again on every invocation. From the second invocation on, the
  test binaries come from the cache, so both test processes are spawned close together.
- More empty test binaries give more chances to overlap.

## Proposed fix

**Pass the handles to inherit explicitly.**

- Build a `STARTUPINFOEXW` whose attribute list holds `PROC_THREAD_ATTRIBUTE_HANDLE_LIST`. The list names the
  child's stdin, stdout and stderr handles, the progress pipe handle, and any handles the caller wants the child
  to inherit. Use `InitializeProcThreadAttributeList` and `UpdateProcThreadAttribute`, and pass
  `EXTENDED_STARTUPINFO_PRESENT` to `CreateProcessW`.
- Once every std spawn uses such a list, a spawn gets only the handles in its own list. It does not matter how
  many inheritable handles other threads hold at that moment. Std spawns then need no lock among themselves, and
  the guarantee no longer depends on which `Io` instance does the spawning.
- Go and Python both work this way:
  - Go: `syscall/exec_windows.go` uses `_PROC_THREAD_ATTRIBUTE_HANDLE_LIST` ("Do not accidentally inherit more
    than these handles"); see [golang.org/issue/44011](https://golang.org/issue/44011).
  - Python: `subprocess.Popen` with `close_fds=True` fills `lpAttributeList["handle_list"]`.
- Raymond Chen describes the technique in
  [The Old New Thing, 2011-12-16](https://devblogs.microsoft.com/oldnewthing/20111216-00/?p=8873).

Caveats of the handle list:

- The listed handles must still be inheritable.
- A `CreateProcessW` call that passes no list, for example from C code in the same process, still inherits every
  inheritable handle. To narrow that window:
  - create the handles non-inheritable and make them inheritable only for the spawn, as 0.17.0 does for
    `inherit_files`; or
  - duplicate them as inheritable just before `CreateProcessW` and close the duplicates right after.
- Leave NULL handles out of the list. Go's comment: "The presence of a NULL handle in the list is enough to
  cause PROC_THREAD_ATTRIBUTE_HANDLE_LIST to treat the entire list as empty".

**Alternative: a process-wide lock.** Hold it from the creation of the inheritable pipe ends until
`CreateProcessW` has returned and the parent has closed its copies of the child ends. This is simpler, but it
serializes all spawns, and it only protects spawns that take the lock.

## Possible follow-up for 0.17.0 and later

0.17.0 implements the alternative, but the mutex lives in the `Io.Threaded` instance (`t.process_spawn_mutex`).
Two gaps remain:

- Spawns through two different `Io.Threaded` instances in one process do not share the mutex. They can still
  inherit each other's pipe ends.
- `CreateProcessW` still passes `bInheritHandles = TRUE` without a handle list (0.17.0 `Threaded.zig`, line
  16757 onwards). Each child therefore still inherits every handle that is inheritable in the process at that
  moment, including handles that other code made inheritable.

`PROC_THREAD_ATTRIBUTE_HANDLE_LIST` would close both gaps. This fits the open discussion in Codeberg
ziglang/zig#30727 ("`std.process.Child` and handles") about which handles a child process inherits.

## References

- Raymond Chen, The Old New Thing, 2011-12-16, on choosing which handles a new process inherits:
  https://devblogs.microsoft.com/oldnewthing/20111216-00/?p=8873
- Go issue 44011, "syscall: add additional fields to SysProcAttr on Windows": https://golang.org/issue/44011
  (the handle list code is in `src/syscall/exec_windows.go`)
- Python `Lib/subprocess.py`, `Popen._execute_child` on Windows (`handle_list`)
- Microsoft, `UpdateProcThreadAttribute`:
  https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-updateprocthreadattribute
- Codeberg ziglang/zig#30727, "`std.process.Child` and handles": https://codeberg.org/ziglang/zig/issues/30727
