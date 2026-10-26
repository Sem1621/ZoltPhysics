//! Port of: PerformanceTest/PerformanceTest.cpp
//! Status: complete
//! Not ported: Jolt's profiler (JPH_PROFILE_START / NEXTFRAME / DUMP / END are compiled out as in Jolt's Distribution
//! build), JPH_TRACK_NARROWPHASE_STATS and the Android entry point.
//!
//! Milestone M6 (SimulationArchitecture section 3): `zig build performance-test -Doptimize=ReleaseFast -- -q=LinearCast
//! -t=max -s=Pyramid -validate_hash=0x74d0118836ac0892` must print Jolt's published hash (determinism_check.yml), in
//! both precisions and for every thread count. Milestone M8 (CharacterVehicleArchitecture section 1): the same for
//! `-q=Discrete -t=max -s=CharacterVirtual -validate_hash=0xc1d4e05b164b1221` (0xa0b16727aa16fdb0 in double precision).
//! Milestone M9 (SerializationArchitecture section 1): the same for `-q=LinearCast -t=max -s=Ragdoll
//! -validate_hash=0x3be6ea76c07a2ac8` (0x8dd6bc501859a228 in double precision). Ragdoll is the default scene, like in
//! a Jolt build with JPH_OBJECT_STREAM.
//!
//! The program is a statement by statement port of PerformanceTest.cpp, with the same arguments, the same output text
//! and the same hash:
//! - `Trace` is `std.log.info`; `std_options.logFn` is Jolt's TraceImpl: every message (also the library's) is
//!   formatted into a 1024 byte buffer (longer messages are truncated like vsnprintf) and printed to stdout with a new
//!   line. printf formats become Zig formats (`%f` is `{d:.6}`, with printf's texts for NaN and infinity). Zig rounds
//!   the shortest round trip digits of the value where printf rounds its exact binary value (ties to even), so a value
//!   whose shortest digits end halfway can differ in the last printed digit (0.1234565: Zig 0.123457, printf
//!   0.123456). This only affects printed timings and LargeMeshScene's statistics (`%.1f` / `%.3f`), not the hash.
//! - JPH_BREAKPOINT in StateRecorderImpl::ReadBytes (-vs): Jolt stops the program at the first mismatch in every build
//!   (int3 / __builtin_trap, not tied to JPH_ENABLE_ASSERTS), Zolt's StateRecorderImpl only panics when asserts are on.
//!   `validateState` reports a mismatch (`num_mismatches`) or a read past the end of the state (a shorter or missing
//!   state file: Jolt compares against uninitialized stack memory) and main then runs `@breakpoint()` (int3 / brk,
//!   SIGTRAP like Jolt) after the state of that frame is restored. Without asserts the trace therefore lists all
//!   mismatches of the frame where Jolt's lists the first one; with asserts the library panics at the first one.
//! - `RegisterDefaultAllocator`, `Factory::sInstance`, `RegisterTypes` / `UnregisterTypes`: nothing to do, the
//!   allocator is `init.gpa` and the type registry of Zolt is comptime.
//! - The scenes are PerformanceTestScene interfaces (`unique_ptr<PerformanceTestScene>`: the interface owns the scene).
//! - `thread::hardware_concurrency()` is `std.Thread.getCpuCount()` (0 when it fails), `atoi` is C's atoi.
//! - `std::chrono::high_resolution_clock` is the monotonic clock of `std.Io` (`Clock.awake`).
//! - `-r` (JPH_DEBUG_RENDERER, `-Ddebug_renderer=true`; without the option the argument is ignored, like a Jolt build
//!   without the debug renderer): every update draws the bodies into a DebugRendererRecorder that writes
//!   `performance_test_<tag>.jor`, and ends the frame. The recorder is the renderer of the physics system
//!   (`setDebugRenderer`, where Jolt's recorder constructor sets DebugRenderer::sInstance, RemainingFeaturesArchitecture.md
//!   R4). Jolt constructs the recorder without `-r` too, writing into a file that is not open. No draw flag is set then,
//!   so nothing is drawn: Zolt only creates the recorder with `-r`. `zig build debug-renderer-check
//!   -Ddebug_renderer=true` compares the .jor bytes with the C++ PerformanceTest.
//! - `std::filesystem::path` operations of the asset search are emulated on strings (`parentPath`, `appendPath`), the
//!   `ofstream` / `ifstream` of the per frame timings and the state files are `OutputFile` / `InputFile` (a file that
//!   could not be opened ignores writes and reads nothing, like a failed stream).
//! - JPH_DET_LOG is `if (DeterminismLog.enabled) DeterminismLog.log(.{ ... })` (build option `-Ddeterminism_log`).
//! - `-p` (Zolt, in place of Jolt's profile dumps): with `-Djob_profile=true` the thread pool records the jobs and the
//!   idle time of every run (Zolt/Core/JobProfiler.zig, from the first update to the last). After the stat line of a
//!   run, `-p` writes the timeline as a Chrome trace `<scene>_<tag>.json` in the current directory (e.g.
//!   `Ragdoll_linearcast_th4.json`, open it in chrome://tracing or https://ui.perfetto.dev) and prints one line with
//!   the wall time and the busy and idle time of each thread. Without the option `-p` prints a note. With `-p` each
//!   thread keeps up to `profile_capacity` events (about 10 MB per thread), enough for the default 500 steps; a longer
//!   run loses its oldest events, which the trace and tools/job_trace_stats.py report.
//! - `-phase_times` (Zolt): after the stat line of a run, print the total time of the scene updates (UpdateTest) and of
//!   the physics updates (PhysicsSystem::Update), in milliseconds. The hash does not change.

const std = @import("std");
const zolt = @import("zolt");
const PerformanceTestScene = @import("PerformanceTestScene.zig").PerformanceTestScene;
const LayersFile = @import("Layers.zig");
const ConvexVsMeshScene = @import("ConvexVsMeshScene.zig").ConvexVsMeshScene;
const PyramidScene = @import("PyramidScene.zig").PyramidScene;
const LargeMeshScene = @import("LargeMeshScene.zig").LargeMeshScene;
const MaxBodiesScene = @import("MaxBodiesScene.zig").MaxBodiesScene;
const HighSpeedScene = @import("HighSpeedScene.zig").HighSpeedScene;
const CharacterVirtualScene = @import("CharacterVirtualScene.zig").CharacterVirtualScene;
const RagdollScene = @import("RagdollScene.zig").RagdollScene;

const Allocator = std.mem.Allocator;
const Real = zolt.Real;
const MotionQuality = zolt.MotionQuality;
const PhysicsSystem = zolt.PhysicsSystem;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const JobSystemThreadPool = zolt.JobSystemThreadPool;
const StateRecorderImpl = zolt.StateRecorderImpl;
const BodyIDVector = zolt.BodyIDVector;
const BodyLockRead = zolt.BodyLockRead;
const BodyLockWrite = zolt.BodyLockWrite;
const DeterminismLog = zolt.DeterminismLog;
const HashCombine = zolt.HashCombine;
const physics_settings = zolt.physics_settings;

const log = std.log;

/// Time step for physics
const delta_time: f32 = 1.0 / 60.0;

/// Zolt: events per thread of the job profiler with `-p` (Core/JobProfiler.zig, 40 bytes each)
const profile_capacity: u32 = 256 * 1024;

/// Jolt's `Trace = TraceImpl`: every log message of the program and the library goes to stdout (Trace has no levels)
pub const std_options: std.Options = .{
    .log_level = .info,
    .logFn = traceImpl,
};

/// TraceImpl
fn traceImpl(comptime level: std.log.Level, comptime scope: @EnumLiteral(), comptime format: []const u8, args: anytype) void {
    _ = level;
    _ = scope;

    // Format the message (vsnprintf into a char[1024]: at most 1023 characters)
    var buffer: [1024]u8 = undefined;
    var writer: std.Io.Writer = .fixed(buffer[0 .. buffer.len - 1]);
    writer.print(format, args) catch {}; // A fixed writer fills the buffer before it fails: the message is truncated
    const len = writer.end;

    // Print to the TTY (cout << buffer << endl)
    buffer[len] = '\n';
    std.Io.File.stdout().writeStreamingAll(std.Options.debug_io, buffer[0 .. len + 1]) catch {};
}

/// printf("%f", value): `{d:.6}`, with printf's texts for NaN and infinity. Equal to printf except for values whose
/// shortest round trip digits end halfway between two 6 digit decimals: Zig rounds those digits, printf the exact
/// binary value (0.1234565: Zig 0.123457, printf 0.123456)
const PrintfFloat = struct {
    value: f64,

    pub fn format(self: PrintfFloat, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        if (std.math.isNan(self.value))
            return writer.writeAll(if (std.math.signbit(self.value)) "-nan" else "nan");
        if (std.math.isInf(self.value))
            return writer.writeAll(if (self.value < 0) "-inf" else "inf");
        return writer.print("{d:.6}", .{self.value});
    }
};

/// C's atoi (glibc: `(int)strtol(s, NULL, 10)`): leading white space, an optional sign and decimal digits, 0 when there
/// are no digits; strtol saturates at the limits of `long`, the conversion to int keeps the lower 32 bits
fn atoi(s: []const u8) i32 {
    var i: usize = 0;
    while (i < s.len and std.ascii.isWhitespace(s[i])) i += 1;
    var negative = false;
    if (i < s.len and (s[i] == '+' or s[i] == '-')) {
        negative = s[i] == '-';
        i += 1;
    }
    const limit: i128 = @as(i128, std.math.maxInt(i64)) + 1;
    var value: i128 = 0;
    while (i < s.len and std.ascii.isDigit(s[i])) : (i += 1)
        value = @min(value * 10 + (s[i] - '0'), limit);
    if (negative) value = -value;
    const long: i64 = @intCast(std.math.clamp(value, std.math.minInt(i64), std.math.maxInt(i64)));
    return @truncate(long);
}

/// thread::hardware_concurrency(): the number of CPUs, 0 when it cannot be determined
fn hardwareConcurrency() u32 {
    const count = std.Thread.getCpuCount() catch return 0;
    return std.math.cast(u32, count) orelse std.math.maxInt(u32);
}

/// std::filesystem::path::root_path() of a path (the leading separator, if any)
fn rootPath(path: []const u8) []const u8 {
    return if (path.len > 0 and std.fs.path.isSep(path[0])) path[0..1] else path[0..0];
}

/// std::filesystem::path::parent_path(): the path without its last element and the separators before it (the path
/// itself when it has no relative part)
fn parentPath(path: []const u8) []const u8 {
    const root = rootPath(path);
    if (path.len == root.len)
        return path;
    var end = path.len;
    while (end > root.len and !std.fs.path.isSep(path[end - 1])) end -= 1; // The file name
    while (end > root.len and std.fs.path.isSep(path[end - 1])) end -= 1; // The separators before it
    return path[0..end];
}

/// std::filesystem::path::operator / (a relative element): adds a separator unless the path is empty or ends with one
fn appendPath(allocator: Allocator, path: []const u8, element: []const u8) Allocator.Error![]u8 {
    if (path.len == 0)
        return allocator.dupe(u8, element);
    if (std.fs.path.isSep(path[path.len - 1]))
        return std.mem.concat(allocator, u8, &.{ path, element });
    return std.mem.concat(allocator, u8, &.{ path, "/", element });
}

/// std::filesystem::exists
fn exists(io: std.Io, path: []const u8) bool {
    std.Io.Dir.cwd().access(io, path, .{}) catch return false;
    return true;
}

/// An std::ofstream opened for writing (truncated): when the file cannot be opened the writes are ignored, write errors
/// are ignored too (the stream state is never checked). Initialized in place (the writer points into `buffer`).
const OutputFile = struct {
    io: std.Io = undefined,
    file: ?std.Io.File = null,
    file_writer: std.Io.File.Writer = undefined,
    buffer: [4096]u8 = undefined,

    fn open(self: *OutputFile, io: std.Io, name: []const u8) void {
        self.io = io;
        self.file = std.Io.Dir.cwd().createFile(io, name, .{}) catch null;
        if (self.file) |f|
            self.file_writer = f.writer(io, &self.buffer);
    }

    fn writer(self: *OutputFile) ?*std.Io.Writer {
        return if (self.file != null) &self.file_writer.interface else null;
    }

    fn write(self: *OutputFile, bytes: []const u8) void {
        if (self.writer()) |w| w.writeAll(bytes) catch {};
    }

    fn print(self: *OutputFile, comptime format: []const u8, args: anytype) void {
        if (self.writer()) |w| w.print(format, args) catch {};
    }

    /// The destructor: flush and close
    fn close(self: *OutputFile) void {
        if (self.file) |f| {
            self.file_writer.interface.flush() catch {};
            f.close(self.io);
            self.file = null;
        }
    }
};

/// An std::ifstream opened for reading: `read` reads up to the size of the buffer, a short read sets the fail state,
/// after which (and when the file cannot be opened) reads read nothing. Initialized in place (the reader points into
/// `buffer`).
const InputFile = struct {
    io: std.Io = undefined,
    file: ?std.Io.File = null,
    file_reader: std.Io.File.Reader = undefined,
    buffer: [4096]u8 = undefined,
    failed: bool = false,

    fn open(self: *InputFile, io: std.Io, name: []const u8) void {
        self.io = io;
        self.file = std.Io.Dir.cwd().openFile(io, name, .{}) catch null;
        if (self.file) |f|
            self.file_reader = f.reader(io, &self.buffer)
        else
            self.failed = true;
    }

    fn read(self: *InputFile, out: []u8) void {
        if (self.failed)
            return;
        const num_read = self.file_reader.interface.readSliceShort(out) catch 0;
        if (num_read < out.len)
            self.failed = true;
    }

    /// The destructor
    fn close(self: *InputFile) void {
        if (self.file) |f| {
            f.close(self.io);
            self.file = null;
        }
    }
};

/// The -vs branch of main after the state of a frame was read: copy it to a validator and validate the physics system
/// against it. Zolt: returns true where Jolt's StateRecorderImpl::ReadBytes runs JPH_BREAKPOINT (the caller breaks):
/// the validator counted a mismatch, or it read past the end of the state (Jolt then compares against uninitialized
/// stack memory, Zolt's validator reads nothing and is left failed).
fn validateState(allocator: Allocator, physics_system: *PhysicsSystem, data: []const u8) Allocator.Error!bool {
    // Copy to validator
    var validator = StateRecorderImpl.init(allocator);
    defer validator.deinit();
    validator.writeBytes(data);
    if (validator.isFailed())
        return error.OutOfMemory; // The stringstream could not grow (Jolt: std::bad_alloc)

    // Validate state
    validator.recorder().setValidating(true);
    _ = try physics_system.restoreState(validator.recorder(), .{});

    return validator.num_mismatches != 0 or validator.isFailed();
}

/// Zolt, -p with -Djob_profile=true (see the file header): stop the job profiler of the run and the worker threads
/// (a worker can still record the end of a wait), write the Chrome trace into the file `name` in `dir` and the summary
/// line to `summary`
fn writeJobProfile(io: std.Io, job_system: *JobSystemThreadPool, dir: std.Io.Dir, name: []const u8, process_name: []const u8, summary: *std.Io.Writer) !void {
    const profiler = job_system.getJobProfiler();
    profiler.stop();
    try job_system.setNumThreads(0);

    const file = try dir.createFile(io, name, .{});
    defer file.close(io);
    var buffer: [4096]u8 = undefined;
    var file_writer = file.writer(io, &buffer);
    try profiler.writeChromeTrace(&file_writer.interface, process_name);
    try file_writer.interface.flush();

    try profiler.writeSummary(summary);
}

/// The helper of main that creates the default scene: a RagdollScene(4, 10, 0.6f) (Jolt: with JPH_OBJECT_STREAM, which
/// Zolt always has, SerializationArchitecture S1)
fn createRagdollScene(allocator: Allocator, io: std.Io) Allocator.Error!PerformanceTestScene {
    return (try RagdollScene.create(allocator, io, 4, 10, 0.6)).scene();
}

/// Replace the scene (unique_ptr assignment: the old scene is destroyed)
fn setScene(scene: *?PerformanceTestScene, new_scene: PerformanceTestScene) void {
    if (scene.*) |old| old.destroy();
    scene.* = new_scene;
}

// Program entry point
pub fn main(init: std.process.Init) !u8 {
    const allocator = init.gpa;
    const io = init.io;
    const arena = init.arena.allocator();
    const argv = try init.minimal.args.toSlice(arena);

    // Install callbacks (Trace = TraceImpl: std_options.logFn)

    // Register allocation hook (Zolt: the allocator is passed explicitly)

    // Helper function that creates the default scene: createRagdollScene

    // Parse command line parameters
    var specified_quality: i32 = -1;
    var specified_threads: i32 = -1;
    var max_iterations: u32 = 500;
    var disable_sleep = false;
    var enable_profiler = false;
    var enable_phase_times = false; // Zolt: -phase_times
    var enable_debug_renderer = false; // JPH_DEBUG_RENDERER
    var enable_per_frame_recording = false;
    var record_state = false;
    var validate_state = false;
    var scene: ?PerformanceTestScene = null;
    defer if (scene) |s| s.destroy();
    var validate_hash: ?[]const u8 = null;
    var repeat: i32 = 1;
    for (argv[@min(1, argv.len)..]) |arg| {
        if (std.mem.startsWith(u8, arg, "-s=")) {
            // Parse scene
            const name = arg[3..];
            if (std.mem.eql(u8, name, "Ragdoll"))
                setScene(&scene, try createRagdollScene(allocator, io))
            else if (std.mem.eql(u8, name, "RagdollSinglePile"))
                setScene(&scene, (try RagdollScene.create(allocator, io, 1, 160, 0.4)).scene())
            else if (std.mem.eql(u8, name, "ConvexVsMesh"))
                setScene(&scene, (try ConvexVsMeshScene.create(allocator)).scene())
            else if (std.mem.eql(u8, name, "Pyramid"))
                setScene(&scene, (try PyramidScene.create(allocator)).scene())
            else if (std.mem.eql(u8, name, "LargeMesh"))
                setScene(&scene, (try LargeMeshScene.create(allocator, io)).scene())
            else if (std.mem.eql(u8, name, "CharacterVirtual"))
                setScene(&scene, (try CharacterVirtualScene.create(allocator)).scene())
            else if (std.mem.eql(u8, name, "MaxBodies"))
                setScene(&scene, (try MaxBodiesScene.create(allocator)).scene())
            else if (std.mem.eql(u8, name, "HighSpeed"))
                setScene(&scene, (try HighSpeedScene.create(allocator)).scene())
            else {
                log.info("Invalid scene", .{});
                return 1;
            }
        } else if (std.mem.startsWith(u8, arg, "-i=")) {
            // Parse max iterations
            max_iterations = @bitCast(atoi(arg[3..]));
        } else if (std.mem.startsWith(u8, arg, "-q=")) {
            // Parse quality
            if (std.mem.eql(u8, arg[3..], "Discrete"))
                specified_quality = 0
            else if (std.mem.eql(u8, arg[3..], "LinearCast"))
                specified_quality = 1
            else {
                log.info("Invalid quality", .{});
                return 1;
            }
        } else if (std.mem.startsWith(u8, arg, "-t=max")) {
            // Default to number of threads on the system
            specified_threads = @bitCast(hardwareConcurrency());
        } else if (std.mem.startsWith(u8, arg, "-t=")) {
            // Parse threads
            specified_threads = atoi(arg[3..]);
        } else if (std.mem.eql(u8, arg, "-no_sleep")) {
            disable_sleep = true;
        } else if (std.mem.eql(u8, arg, "-p")) {
            enable_profiler = true;
        } else if (std.mem.eql(u8, arg, "-phase_times")) {
            enable_phase_times = true;
        } else if (zolt.Core.debug_renderer and std.mem.eql(u8, arg, "-r")) { // JPH_DEBUG_RENDERER
            enable_debug_renderer = true;
        } else if (std.mem.eql(u8, arg, "-f")) {
            enable_per_frame_recording = true;
        } else if (std.mem.eql(u8, arg, "-rs")) {
            record_state = true;
        } else if (std.mem.eql(u8, arg, "-vs")) {
            validate_state = true;
        } else if (std.mem.startsWith(u8, arg, "-validate_hash=")) {
            validate_hash = arg[15..];
        } else if (std.mem.startsWith(u8, arg, "-repeat=")) {
            // Parse repeat count
            repeat = atoi(arg[8..]);
        } else if (std.mem.eql(u8, arg, "-h")) {
            // Print usage
            log.info("Usage:\n" ++
                "-s=<scene>: Select scene (Ragdoll, RagdollSinglePile, ConvexVsMesh, Pyramid)\n" ++
                "-i=<num physics steps>: Number of physics steps to simulate (default 500)\n" ++
                "-q=<quality>: Test only with specified quality (Discrete, LinearCast)\n" ++
                "-t=<num threads>: Test only with N threads (default is to iterate over 1 .. num hardware threads)\n" ++
                "-t=max: Test with the number of threads available on the system\n" ++
                "-p: Write out profiles\n" ++
                "-phase_times: Print the time of the scene and the physics updates (Zolt)\n" ++
                "-r: Record debug renderer output for JoltViewer\n" ++
                "-f: Record per frame timings\n" ++
                "-no_sleep: Disable sleeping\n" ++
                "-rs: Record state\n" ++
                "-vs: Validate state\n" ++
                "-validate_hash=<hash>: Validate hash (return 0 if successful, 1 if failed)\n" ++
                "-repeat=<num>: Repeat all tests <num> times", .{});
            return 0;
        }
    }

    // Create a factory, register all Jolt physics types: nothing to do (Zolt's type registry is comptime)

    // Show used instruction sets
    log.info("{s}", .{zolt.getConfigurationString()});

    // If no scene was specified use the default scene
    if (scene == null)
        scene = try createRagdollScene(allocator, io);
    const s = scene.?;

    // Output scene we're running
    log.info("Running scene: {s}", .{s.getName()});

    // Create temp allocator
    var temp_allocator = try TempAllocatorImpl.init(allocator, s.getTempAllocatorSizeMB() * 1024 * 1024);
    defer temp_allocator.deinit();

    // Find the asset path
    var found = false;
    var asset_path: []const u8 = if (argv.len > 0) argv[0] else "";
    const root_path = rootPath(asset_path);
    while (!std.mem.eql(u8, asset_path, root_path)) {
        asset_path = parentPath(asset_path);
        if (exists(io, try appendPath(arena, asset_path, "Assets"))) {
            found = true;
            break;
        }
    }
    if (!found) { // Note that argv[0] can be a relative path like './PerformanceTest' so we also scan up using '..'
        for (0..5) |_| {
            asset_path = try appendPath(arena, asset_path, "..");
            if (exists(io, try appendPath(arena, asset_path, "Assets"))) {
                found = true;
                break;
            }
        }
    }
    if (!found)
        asset_path = "Assets"
    else
        asset_path = try appendPath(arena, asset_path, "Assets");
    asset_path = try appendPath(arena, asset_path, "");

    // Load the scene
    if (!try s.load(asset_path))
        return 1;

    // Create mapping table from object layer to broadphase layer
    const broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl = .init();

    // Create class that filters object vs broadphase layers
    const object_vs_broadphase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{};

    // Create class that filters object vs object layers
    const object_vs_object_layer_filter: LayersFile.ObjectLayerPairFilterImpl = .{};

    // Start profiling this program (JPH_PROFILE_START: compiled out)
    // Zolt: -p writes the job timeline of every run, which needs a build with -Djob_profile=true (see the file header)
    if (enable_profiler and !zolt.Core.job_profile)
        log.info("-p: this build has no job profiler, build with -Djob_profile=true", .{});

    // Start the determinism log
    if (DeterminismLog.enabled) try DeterminismLog.open(io);
    defer if (DeterminismLog.enabled) DeterminismLog.close();

    // Trace header
    log.info("Motion Quality, Thread Count, Steps / Second, Hash", .{});

    // Repeat test
    var r: i32 = 0;
    while (r < repeat) : (r += 1) {
        // Iterate motion qualities
        for (0..2) |mq| {
            // Skip quality if another was specified
            if (specified_quality != -1 and mq != @as(u32, @bitCast(specified_quality)))
                continue;

            // Determine motion quality
            const motion_quality: MotionQuality = if (mq == 0) .discrete else .linear_cast;
            const motion_quality_str: []const u8 = if (mq == 0) "Discrete" else "LinearCast";

            // Determine which thread counts to test
            var thread_permutations: std.ArrayList(u32) = .empty;
            defer thread_permutations.deinit(allocator);
            if (specified_threads > 0) {
                try thread_permutations.append(allocator, @as(u32, @bitCast(specified_threads)) - 1);
            } else {
                var num_threads: u32 = 0;
                while (num_threads < hardwareConcurrency()) : (num_threads += 1)
                    try thread_permutations.append(allocator, num_threads);
            }

            // Test thread permutations
            for (thread_permutations.items) |num_threads| {
                // Create job system with desired number of threads
                var job_system: JobSystemThreadPool = .empty;
                try job_system.init(allocator, io, physics_settings.max_physics_jobs, physics_settings.max_physics_barriers, .{
                    .num_threads = @bitCast(num_threads),
                    // Zolt: with -p the trace holds the whole run of 500 steps (about 200 events per step and thread)
                    .profile_capacity = if (enable_profiler) profile_capacity else zolt.JobProfiler.default_capacity,
                });
                defer job_system.deinit();

                // Create physics system
                var physics_system: PhysicsSystem = .empty;
                try physics_system.init(allocator, io, s.getMaxBodies(), 0, s.getMaxBodyPairs(), s.getMaxContactConstraints(), &broad_phase_layer_interface.base, &object_vs_broadphase_layer_filter.base, &object_vs_object_layer_filter.base, .{});
                defer physics_system.deinit();

                // Start test scene
                try s.startTest(&physics_system, motion_quality);
                errdefer s.stopTest(&physics_system) catch {}; // Zolt: on an error the scene's objects leave the system before it is destroyed

                // Disable sleeping if requested
                if (disable_sleep) {
                    const bli = physics_system.getBodyLockInterfaceNoLock();
                    var body_ids: BodyIDVector = .empty;
                    defer body_ids.deinit(allocator);
                    try physics_system.getBodies(allocator, &body_ids);
                    for (body_ids.items) |id| {
                        var lock: BodyLockWrite = .init(bli, id);
                        defer lock.deinit();
                        if (lock.succeeded()) {
                            const body = lock.getBody();
                            if (!body.isStatic())
                                body.setAllowSleeping(false);
                        }
                    }
                }

                // Optimize the broadphase to prevent an expensive first frame
                try physics_system.optimizeBroadPhase();

                // A tag used to identify the test
                const motion_quality_lower = try zolt.toLower(allocator, motion_quality_str);
                defer allocator.free(motion_quality_lower);
                const num_threads_str = try zolt.convertToString(allocator, num_threads + 1);
                defer allocator.free(num_threads_str);
                const tag = try std.mem.concat(allocator, u8, &.{ motion_quality_lower, "_th", num_threads_str });
                defer allocator.free(tag);

                // Open renderer output (JPH_DEBUG_RENDERER)
                var renderer_file: OutputFile = .{};
                defer renderer_file.close();
                var discard_buffer: [64]u8 = undefined;
                var discarding: std.Io.Writer.Discarding = .init(&discard_buffer); // A file that is not open ignores writes
                var renderer_stream: zolt.StreamOutWrapper = undefined;
                var renderer: if (zolt.Core.debug_renderer) zolt.DebugRendererRecorder else void = undefined;
                if (zolt.Core.debug_renderer and enable_debug_renderer) {
                    const name = try std.mem.concat(allocator, u8, &.{ "performance_test_", tag, ".jor" });
                    defer allocator.free(name);
                    renderer_file.open(io, name);
                    renderer_stream = .init(renderer_file.writer() orelse &discarding.writer);
                    try renderer.init(allocator, io, renderer_stream.streamOut());
                    physics_system.setDebugRenderer(renderer.debugRenderer());
                }
                defer if (zolt.Core.debug_renderer and enable_debug_renderer) {
                    physics_system.setDebugRenderer(null);
                    renderer.deinit();
                };

                // Open per frame timing output
                var per_frame_file: OutputFile = .{};
                defer per_frame_file.close();
                if (enable_per_frame_recording) {
                    const name = try std.mem.concat(allocator, u8, &.{ "per_frame_", tag, ".csv" });
                    defer allocator.free(name);
                    per_frame_file.open(io, name);
                    per_frame_file.write("Frame, Time (ms)\n");
                }

                var record_state_file: OutputFile = .{};
                defer record_state_file.close();
                var validate_state_file: InputFile = .{};
                defer validate_state_file.close();
                if (record_state or validate_state) {
                    const name = try std.mem.concat(allocator, u8, &.{ "state_", motion_quality_lower, ".bin" });
                    defer allocator.free(name);
                    if (record_state)
                        record_state_file.open(io, name)
                    else if (validate_state)
                        validate_state_file.open(io, name);
                }

                var total_duration: i96 = 0;

                // Zolt: -phase_times, the total time of the scene updates and of the physics updates
                var scene_update_duration: i96 = 0;
                var physics_update_duration: i96 = 0;

                // Zolt: the job timeline of this run starts now (-Djob_profile=true)
                if (zolt.Core.job_profile) job_system.getJobProfiler().reset();

                // Step the world for a fixed amount of iterations
                var iterations: u32 = 0;
                while (iterations < max_iterations) : (iterations += 1) {
                    // JPH_PROFILE_NEXTFRAME: compiled out
                    if (DeterminismLog.enabled) DeterminismLog.log(.{ "Iteration: ", iterations });

                    // Start measuring
                    const clock_start = std.Io.Clock.awake.now(io);

                    // Update the test
                    try s.updateTest(&physics_system, temp_allocator.tempAllocator(), delta_time);

                    // Zolt: -phase_times splits the measurement here
                    const clock_physics = if (enable_phase_times) std.Io.Clock.awake.now(io) else clock_start;

                    // Do a physics step. Zolt: never inlined, so that the binary has the symbol
                    // Physics.PhysicsSystem.PhysicsSystem.update, which `zig build perf-budget` counts with callgrind
                    _ = try @call(.never_inline, PhysicsSystem.update, .{ &physics_system, delta_time, 1, temp_allocator.tempAllocator(), job_system.jobSystem() });

                    // Stop measuring
                    const clock_end = std.Io.Clock.awake.now(io);
                    const duration = clock_start.durationTo(clock_end).toNanoseconds();
                    total_duration += duration;
                    if (enable_phase_times) {
                        scene_update_duration += clock_start.durationTo(clock_physics).toNanoseconds();
                        physics_update_duration += clock_physics.durationTo(clock_end).toNanoseconds();
                    }

                    if (zolt.Core.debug_renderer and enable_debug_renderer) {
                        // Draw the state of the world
                        const settings: zolt.BodyManager.DrawSettings = .{};
                        physics_system.drawBodies(&settings, renderer.debugRenderer(), .{});

                        // Mark end of frame
                        try renderer.endFrame();
                    }

                    // Record time taken this iteration
                    if (enable_per_frame_recording) {
                        const time_str = try zolt.convertToString(allocator, 1.0e-6 * @as(f64, @floatFromInt(duration)));
                        defer allocator.free(time_str);
                        per_frame_file.print("{d}, {s}\n", .{ iterations, time_str });
                    }

                    // Dump profile information every 100 iterations
                    if (enable_profiler and iterations % 100 == 0) {
                        // JPH_PROFILE_DUMP(tag + "_it" + ConvertToString(iterations)): compiled out
                    }

                    if (record_state) {
                        // Record state
                        var recorder = StateRecorderImpl.init(allocator);
                        defer recorder.deinit();
                        try physics_system.saveState(recorder.recorder(), .{});

                        // Write to file
                        const data = recorder.getData();
                        const size: u32 = @truncate(data.len);
                        record_state_file.write(std.mem.asBytes(&size));
                        record_state_file.write(data[0..size]);
                    } else if (validate_state) {
                        // Read state
                        var size: u32 = 0;
                        validate_state_file.read(std.mem.asBytes(&size));
                        const data = try allocator.alloc(u8, size);
                        defer allocator.free(data);
                        @memset(data, 0);
                        validate_state_file.read(data);

                        // Copy to validator and validate state
                        if (try validateState(allocator, &physics_system, data))
                            @breakpoint(); // JPH_BREAKPOINT of StateRecorderImpl::ReadBytes (see the file header)
                    }

                    if (DeterminismLog.enabled) {
                        const bli = physics_system.getBodyLockInterfaceNoLock();
                        var body_ids: BodyIDVector = .empty;
                        defer body_ids.deinit(allocator);
                        try physics_system.getBodies(allocator, &body_ids);
                        for (body_ids.items) |id| {
                            var lock: BodyLockRead = .init(bli, id);
                            defer lock.deinit();
                            const body = lock.getBody();
                            if (!body.isStatic())
                                DeterminismLog.log(.{ id, ": p: ", body.getPosition(), " r: ", body.getRotation(), " v: ", body.getLinearVelocity(), " w: ", body.getAngularVelocity() });
                        }
                    }
                }

                // Zolt: the job timeline of this run ends here (-Djob_profile=true)
                if (zolt.Core.job_profile) job_system.getJobProfiler().stop();

                // Calculate hash of all positions and rotations of the bodies
                var hash: u64 = HashCombine.hashBytes(&.{}); // Ensure we start with the proper seed
                const bi = physics_system.getBodyInterfaceNoLock();
                var body_ids: BodyIDVector = .empty;
                defer body_ids.deinit(allocator);
                try physics_system.getBodies(allocator, &body_ids);
                for (body_ids.items) |id| {
                    const pos = bi.getPosition(id);
                    hash = HashCombine.hashBytesSeeded(std.mem.asBytes(&pos)[0 .. 3 * @sizeOf(Real)], hash);
                    const rot = bi.getRotation(id);
                    hash = HashCombine.hashBytesSeeded(std.mem.asBytes(&rot), hash);
                }

                // Let the scene hash its own state
                s.updateHash(&hash);

                // Convert hash to string
                const hash_str = try std.fmt.allocPrint(allocator, "0x{x}", .{hash});
                defer allocator.free(hash_str);

                // Stop test scene
                try s.stopTest(&physics_system);

                // Trace stat line
                log.info("{s}, {d}, {f}, {s}", .{ motion_quality_str, num_threads + 1, PrintfFloat{ .value = @as(f64, @floatFromInt(max_iterations)) / (1.0e-9 * @as(f64, @floatFromInt(total_duration))) }, hash_str });

                // Zolt: -phase_times
                if (enable_phase_times)
                    log.info("Phase times: scene update {d:.3} ms, physics update {d:.3} ms", .{ 1.0e-6 * @as(f64, @floatFromInt(scene_update_duration)), 1.0e-6 * @as(f64, @floatFromInt(physics_update_duration)) });

                // Zolt: -p writes the job timeline of this run (-Djob_profile=true, see the file header)
                if (zolt.Core.job_profile and enable_profiler) {
                    const name = try std.mem.concat(allocator, u8, &.{ s.getName(), "_", tag, ".json" });
                    defer allocator.free(name);
                    const process_name = try std.mem.concat(allocator, u8, &.{ "ZoltPerformanceTest ", s.getName(), " ", tag });
                    defer allocator.free(process_name);
                    var summary: std.Io.Writer.Allocating = .init(allocator);
                    defer summary.deinit();
                    if (writeJobProfile(io, &job_system, std.Io.Dir.cwd(), name, process_name, &summary.writer)) {
                        // The summary has one entry per thread: it can be longer than a Trace line (1023 characters)
                        try summary.writer.writeByte('\n');
                        std.Io.File.stdout().writeStreamingAll(io, summary.written()) catch {};
                    } else |err| {
                        log.info("-p: could not write {s}: {t}", .{ name, err });
                    }
                }

                // Check hash code
                if (validate_hash) |expected| {
                    if (!std.mem.eql(u8, hash_str, expected)) {
                        log.info("Fail hash validation. Was: {s}, expected: {s}", .{ hash_str, expected });
                        return 1;
                    }
                }
            }
        }
    }

    // JPH_TRACK_NARROWPHASE_STATS (NarrowPhaseStat::sReportStats): not ported

    // Unregisters all types with the factory and cleans up the default material, destroy the factory: nothing to do

    // End the determinism log (the deferred DeterminismLog.close)

    // End profiling this program (JPH_PROFILE_END: compiled out)

    return 0;
}

test "atoi" {
    try std.testing.expectEqual(@as(i32, 0), atoi(""));
    try std.testing.expectEqual(@as(i32, 0), atoi("max"));
    try std.testing.expectEqual(@as(i32, 42), atoi("42"));
    try std.testing.expectEqual(@as(i32, 42), atoi("  42abc"));
    try std.testing.expectEqual(@as(i32, -7), atoi("-7"));
    try std.testing.expectEqual(@as(i32, 3), atoi("+3"));
    try std.testing.expectEqual(@as(i32, -1), atoi("9223372036854775807")); // LONG_MAX truncated to int
    try std.testing.expectEqual(@as(i32, -1), atoi("99999999999999999999")); // strtol saturates at LONG_MAX
}

test "path helpers emulate std::filesystem::path" {
    try std.testing.expectEqualStrings("/", rootPath("/a/b"));
    try std.testing.expectEqualStrings("", rootPath("a/b"));
    try std.testing.expectEqualStrings("/a", parentPath("/a/b"));
    try std.testing.expectEqualStrings("/", parentPath("/a"));
    try std.testing.expectEqualStrings("/", parentPath("/"));
    try std.testing.expectEqualStrings("a/b", parentPath("a/b/"));
    try std.testing.expectEqualStrings(".", parentPath("./PerformanceTest"));
    try std.testing.expectEqualStrings("", parentPath("."));
    try std.testing.expectEqualStrings("", parentPath("PerformanceTest"));

    const allocator = std.testing.allocator;
    const cases = [_][3][]const u8{
        .{ "", "Assets", "Assets" },
        .{ "a", "Assets", "a/Assets" },
        .{ "a/", "Assets", "a/Assets" },
        .{ "/", "..", "/.." },
        .{ "a/Assets", "", "a/Assets/" },
    };
    for (cases) |c| {
        const joined = try appendPath(allocator, c[0], c[1]);
        defer allocator.free(joined);
        try std.testing.expectEqualStrings(c[2], joined);
    }
}

test "PrintfFloat" {
    var buffer: [64]u8 = undefined;
    try std.testing.expectEqualStrings("123.456789", try std.fmt.bufPrint(&buffer, "{f}", .{PrintfFloat{ .value = 123.4567891 }}));
    try std.testing.expectEqualStrings("0.500000", try std.fmt.bufPrint(&buffer, "{f}", .{PrintfFloat{ .value = 0.5 }}));
    try std.testing.expectEqualStrings("inf", try std.fmt.bufPrint(&buffer, "{f}", .{PrintfFloat{ .value = std.math.inf(f64) }}));
    try std.testing.expectEqualStrings("-nan", try std.fmt.bufPrint(&buffer, "{f}", .{PrintfFloat{ .value = -std.math.nan(f64) }}));
}

/// Zolt test: load a scene, run `num_updates` updates like `main` (with an extra worker thread) and return the hash.
/// The tests run in the repository root (build.zig), so the assets are in "Assets/".
fn runSceneForTest(scene: PerformanceTestScene, motion_quality: MotionQuality, num_updates: u32) !u64 {
    const allocator = std.testing.allocator;
    const io = std.testing.io;
    defer scene.destroy();
    try std.testing.expect(try scene.load("Assets/"));

    var temp_allocator = try TempAllocatorImpl.init(allocator, scene.getTempAllocatorSizeMB() * 1024 * 1024);
    defer temp_allocator.deinit();
    const broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl = .init();
    const object_vs_broadphase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{};
    const object_vs_object_layer_filter: LayersFile.ObjectLayerPairFilterImpl = .{};
    var job_system: JobSystemThreadPool = .empty;
    try job_system.init(allocator, io, physics_settings.max_physics_jobs, physics_settings.max_physics_barriers, .{ .num_threads = 1 });
    defer job_system.deinit();
    var physics_system: PhysicsSystem = .empty;
    try physics_system.init(allocator, io, scene.getMaxBodies(), 0, scene.getMaxBodyPairs(), scene.getMaxContactConstraints(), &broad_phase_layer_interface.base, &object_vs_broadphase_layer_filter.base, &object_vs_object_layer_filter.base, .{});
    defer physics_system.deinit();
    try scene.startTest(&physics_system, motion_quality);
    errdefer scene.stopTest(&physics_system) catch {};
    try physics_system.optimizeBroadPhase();
    for (0..num_updates) |_| {
        try scene.updateTest(&physics_system, temp_allocator.tempAllocator(), delta_time);
        _ = try physics_system.update(delta_time, 1, temp_allocator.tempAllocator(), job_system.jobSystem());
        try std.testing.expect(temp_allocator.isEmpty());
    }

    const hash = try hashForTest(&physics_system, scene);
    try scene.stopTest(&physics_system);
    return hash;
}

/// Zolt test helper: the hash of main (the positions and rotations of all bodies, then the scene's own state)
fn hashForTest(physics_system: *PhysicsSystem, scene: PerformanceTestScene) !u64 {
    const allocator = std.testing.allocator;
    var hash: u64 = HashCombine.hashBytes(&.{});
    const bi = physics_system.getBodyInterfaceNoLock();
    var body_ids: BodyIDVector = .empty;
    defer body_ids.deinit(allocator);
    try physics_system.getBodies(allocator, &body_ids);
    for (body_ids.items) |id| {
        const pos = bi.getPosition(id);
        hash = HashCombine.hashBytesSeeded(std.mem.asBytes(&pos)[0 .. 3 * @sizeOf(Real)], hash);
        const rot = bi.getRotation(id);
        hash = HashCombine.hashBytesSeeded(std.mem.asBytes(&rot), hash);
    }
    scene.updateHash(&hash);
    return hash;
}

/// Zolt test helper: the expected hash of a short run for the build configuration (precision and `-Dfma`)
fn expectedHash(hashes: struct { single: u64, double: u64, fma_single: u64, fma_double: u64 }) u64 {
    if (zolt.Core.use_fmadd)
        return if (zolt.Core.double_precision) hashes.fma_double else hashes.fma_single;
    return if (zolt.Core.double_precision) hashes.double else hashes.single;
}

/// The hash of 10 Pyramid updates (-s=Pyramid -q=LinearCast -i=10)
const pyramid_10_hash = expectedHash(.{ .single = 0x8ae6dee69e292017, .double = 0x874b972e6c896604, .fma_single = 0x9f808de3408d1062, .fma_double = 0x91ac195620392756 });

test "PerformanceTest scenes give Jolt's hashes after a few updates" {
    // The scenes of the determinism check (the published hashes of 500 updates are checked by the determinism CI job,
    // .github/workflows/zolt.yml), a few updates each, in Debug and on every platform of the test CI job. The expected
    // values are the hashes that the C++ PerformanceTest prints with the same arguments (-s=<scene> -q=<quality>
    // -i=<updates> -t=1), built from Jolt/ with CROSS_PLATFORM_DETERMINISTIC. The FMA values (-Dfma=true) come from the
    // C++ PerformanceTest built with -DJPH_USE_FMADD: `zig build jolt-performance-test -Dfma=true
    // [-Ddouble_precision=true] -- <arguments>` on x86-64. The testing allocator also checks that the scenes release all
    // their shapes and settings (for Ragdoll also the objects read from the assets).
    const allocator = std.testing.allocator;
    try std.testing.expectEqual(pyramid_10_hash, try runSceneForTest((try PyramidScene.create(allocator)).scene(), .linear_cast, 10));
    try std.testing.expectEqual(expectedHash(.{ .single = 0xbb1738ab6ae53857, .double = 0x6b13f19597ad79f4, .fma_single = 0x70e604414d2e1e4d, .fma_double = 0x4268bcdd690ddb82 }), try runSceneForTest((try ConvexVsMeshScene.create(allocator)).scene(), .discrete, 100));
    try std.testing.expectEqual(expectedHash(.{ .single = 0x93894e4298eed8e1, .double = 0x2b4f1c7cf1a90e26, .fma_single = 0x6e88572db42e4ae, .fma_double = 0x1225a83f46d8569c }), try runSceneForTest((try ConvexVsMeshScene.create(allocator)).scene(), .linear_cast, 100));
    try std.testing.expectEqual(expectedHash(.{ .single = 0x375628eabc094b31, .double = 0x5109b031ac53c66e, .fma_single = 0x31a8309d7d9e58c4, .fma_double = 0xd41c440313b462d6 }), try runSceneForTest((try HighSpeedScene.create(allocator)).scene(), .linear_cast, 10));
    try std.testing.expectEqual(expectedHash(.{ .single = 0xcb05430a62f9c7f, .double = 0x4ca08591ebc72ba2, .fma_single = 0xcb05430a62f9c7f, .fma_double = 0x4ca08591ebc72ba2 }), try runSceneForTest((try CharacterVirtualScene.create(allocator)).scene(), .discrete, 10));
    // The oracle: zig build jolt-performance-test [-Ddouble_precision=true] [-Dfma=true] -- -q=LinearCast -t=1 -s=Ragdoll -i=5
    try std.testing.expectEqual(expectedHash(.{ .single = 0x47fe0db54d0cdb27, .double = 0x5430d55461b5bdc6, .fma_single = 0x7e335dcf00f4ed8f, .fma_double = 0x349cd7565e3a2c1d }), try runSceneForTest(try createRagdollScene(allocator, std.testing.io), .linear_cast, 5));
}

test "-p writes the job timeline of a run (-Djob_profile)" {
    // Zolt test of -p (see the file header): 10 updates of the Pyramid scene on a pool with a worker thread. The trace
    // is valid JSON and holds the physics jobs under Jolt's job names, the summary names both threads, and the hash is
    // the one of "PerformanceTest scenes give Jolt's hashes after a few updates" (recording changes no result).
    if (!zolt.Core.job_profile) return error.SkipZigTest;
    const allocator = std.testing.allocator;
    const io = std.testing.io;
    const scene = (try PyramidScene.create(allocator)).scene();
    defer scene.destroy();
    try std.testing.expect(try scene.load(""));

    var temp_allocator = try TempAllocatorImpl.init(allocator, scene.getTempAllocatorSizeMB() * 1024 * 1024);
    defer temp_allocator.deinit();
    const broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl = .init();
    const object_vs_broadphase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{};
    const object_vs_object_layer_filter: LayersFile.ObjectLayerPairFilterImpl = .{};
    var job_system: JobSystemThreadPool = .empty;
    try job_system.init(allocator, io, physics_settings.max_physics_jobs, physics_settings.max_physics_barriers, .{ .num_threads = 1 });
    defer job_system.deinit();
    var physics_system: PhysicsSystem = .empty;
    try physics_system.init(allocator, io, scene.getMaxBodies(), 0, scene.getMaxBodyPairs(), scene.getMaxContactConstraints(), &broad_phase_layer_interface.base, &object_vs_broadphase_layer_filter.base, &object_vs_object_layer_filter.base, .{});
    defer physics_system.deinit();
    try scene.startTest(&physics_system, .linear_cast);
    defer scene.stopTest(&physics_system) catch unreachable; // PyramidScene has no StopTest
    try physics_system.optimizeBroadPhase();

    job_system.getJobProfiler().reset();
    for (0..10) |_| {
        try scene.updateTest(&physics_system, temp_allocator.tempAllocator(), delta_time);
        _ = try physics_system.update(delta_time, 1, temp_allocator.tempAllocator(), job_system.jobSystem());
    }
    try std.testing.expectEqual(pyramid_10_hash, try hashForTest(&physics_system, scene));

    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    var summary: std.Io.Writer.Allocating = .init(allocator);
    defer summary.deinit();
    try writeJobProfile(io, &job_system, tmp.dir, "Pyramid_linearcast_th2.json", "ZoltPerformanceTest Pyramid linearcast_th2", &summary.writer);
    try std.testing.expect(!job_system.getJobProfiler().isRecording());
    try std.testing.expectEqual(@as(i32, 1), job_system.getMaxConcurrency()); // The worker threads are stopped

    const trace = try tmp.dir.readFileAlloc(io, "Pyramid_linearcast_th2.json", allocator, .limited(64 * 1024 * 1024));
    defer allocator.free(trace);
    const parsed = try std.json.parseFromSlice(std.json.Value, allocator, trace, .{});
    defer parsed.deinit();
    var num_find_collisions: usize = 0;
    var num_solve_velocity: usize = 0;
    var num_worker_jobs: usize = 0; // Jobs of "Worker 0" (tid 0)
    var num_worker_waits: usize = 0; // "Idle: wait for jobs" of "Worker 0"
    var num_barrier_waits: usize = 0; // "Idle: wait for barrier" of "Thread 0", the thread that calls update (tid 1)
    for (parsed.value.object.get("traceEvents").?.array.items) |event| {
        const name = event.object.get("name").?.string;
        if (std.mem.eql(u8, name, "FindCollisions")) num_find_collisions += 1;
        if (std.mem.eql(u8, name, "SolveVelocityConstraints")) num_solve_velocity += 1;
        if (!std.mem.eql(u8, event.object.get("ph").?.string, "X")) continue;
        const tid = event.object.get("tid").?.integer;
        if (tid == 0 and std.mem.eql(u8, event.object.get("cat").?.string, "job")) num_worker_jobs += 1;
        if (tid == 0 and std.mem.eql(u8, name, "Idle: wait for jobs")) num_worker_waits += 1;
        if (tid == 1 and std.mem.eql(u8, name, "Idle: wait for barrier")) num_barrier_waits += 1;
    }
    try std.testing.expect(num_find_collisions >= 10); // At least one per update
    try std.testing.expect(num_solve_velocity >= 10);

    // The idle spans: the worker runs a job only after it woke up from a wait for jobs, and every update waits at
    // least once for its barrier
    if (num_worker_jobs > 0)
        try std.testing.expect(num_worker_waits > 0);
    try std.testing.expect(num_barrier_waits >= 10);

    try std.testing.expect(std.mem.startsWith(u8, summary.written(), "Job profile: wall "));
    try std.testing.expect(std.mem.indexOf(u8, summary.written(), ", Worker 0: busy ") != null);
    try std.testing.expect(std.mem.indexOf(u8, summary.written(), ", Thread 0: busy ") != null);
}

test "validateState: -vs fails on a state that differs or ends early" {
    // Zolt test of the JPH_BREAKPOINT emulation of -vs (see the file header): record the state of a frame like -rs and
    // validate it like -vs. The same state passes. A state that ends early (a missing or shorter state file) fails, and
    // without asserts (with asserts the library panics at the mismatch) so does the state of another frame.
    const allocator = std.testing.allocator;
    const io = std.testing.io;
    const scene = (try PyramidScene.create(allocator)).scene();
    defer scene.destroy();
    try std.testing.expect(try scene.load(""));

    var temp_allocator = try TempAllocatorImpl.init(allocator, scene.getTempAllocatorSizeMB() * 1024 * 1024);
    defer temp_allocator.deinit();
    const broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl = .init();
    const object_vs_broadphase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{};
    const object_vs_object_layer_filter: LayersFile.ObjectLayerPairFilterImpl = .{};
    var job_system: JobSystemThreadPool = .empty;
    try job_system.init(allocator, io, physics_settings.max_physics_jobs, physics_settings.max_physics_barriers, .{ .num_threads = 1 });
    defer job_system.deinit();
    var physics_system: PhysicsSystem = .empty;
    try physics_system.init(allocator, io, scene.getMaxBodies(), 0, scene.getMaxBodyPairs(), scene.getMaxContactConstraints(), &broad_phase_layer_interface.base, &object_vs_broadphase_layer_filter.base, &object_vs_object_layer_filter.base, .{});
    defer physics_system.deinit();
    try scene.startTest(&physics_system, .linear_cast);
    defer scene.stopTest(&physics_system) catch unreachable; // PyramidScene has no StopTest
    try physics_system.optimizeBroadPhase();
    for (0..3) |_| {
        try scene.updateTest(&physics_system, temp_allocator.tempAllocator(), delta_time);
        _ = try physics_system.update(delta_time, 1, temp_allocator.tempAllocator(), job_system.jobSystem());
    }

    // Record state (-rs)
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    try physics_system.saveState(recorder.recorder(), .{});
    const data = recorder.getData();
    try std.testing.expect(data.len > 0);

    // Validate state (-vs)
    try std.testing.expect(!try validateState(allocator, &physics_system, data));
    try std.testing.expect(try validateState(allocator, &physics_system, &.{})); // Read past the end of the state file
    try std.testing.expect(try validateState(allocator, &physics_system, data[0 .. data.len - 1]));
    try std.testing.expect(!try validateState(allocator, &physics_system, data)); // The failed validations left the state unchanged

    if (!zolt.Core.enable_asserts) {
        // The state of the next frame differs
        try scene.updateTest(&physics_system, temp_allocator.tempAllocator(), delta_time);
        _ = try physics_system.update(delta_time, 1, temp_allocator.tempAllocator(), job_system.jobSystem());
        try std.testing.expect(try validateState(allocator, &physics_system, data));
    }
}

/// Zolt test helper: a physics system for a scene, with the testing allocator (heap allocated, it must not move)
const TestPhysicsSystem = struct {
    broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl,
    object_vs_broadphase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_vs_object_layer_filter: LayersFile.ObjectLayerPairFilterImpl = .{},
    system: PhysicsSystem = .empty,

    fn create(scene: PerformanceTestScene) !*TestPhysicsSystem {
        const allocator = std.testing.allocator;
        const self = try allocator.create(TestPhysicsSystem);
        errdefer allocator.destroy(self);
        self.* = .{ .broad_phase_layer_interface = .init() };
        try self.system.init(allocator, std.testing.io, scene.getMaxBodies(), 0, scene.getMaxBodyPairs(), scene.getMaxContactConstraints(), &self.broad_phase_layer_interface.base, &self.object_vs_broadphase_layer_filter.base, &self.object_vs_object_layer_filter.base, .{});
        return self;
    }

    fn destroy(self: *TestPhysicsSystem) void {
        self.system.deinit();
        std.testing.allocator.destroy(self);
    }
};

/// Zolt test helper: StartTest, OptimizeBroadPhase and StopTest
fn startAndStopForTest(scene: PerformanceTestScene, physics_system: *PhysicsSystem) !void {
    try scene.startTest(physics_system, .linear_cast);
    try physics_system.optimizeBroadPhase();
    try scene.stopTest(physics_system);
}

test "RagdollScene: out of memory while loading, starting and stopping" {
    // Zolt test (porting guide section 10): the allocations of the scene (the objects read from the assets, the
    // ragdolls, the copies of the pose, the lists of RemoveFromPhysicsSystem) fail one at a time. The error must arrive
    // and nothing may leak. Loading allocates thousands of times, so every 997th allocation of Load fails; every
    // allocation of StartTest and StopTest fails. A small scene of 2 x 2 piles of 2 ragdolls keeps it short.
    const io = std.testing.io;

    // Load fails
    var counting = std.testing.FailingAllocator.init(std.testing.allocator, .{});
    {
        const scene = (try RagdollScene.create(counting.allocator(), io, 2, 2, 0.4)).scene();
        defer scene.destroy();
        try std.testing.expect(try scene.load("Assets/"));
    }
    const num_load_allocations = counting.alloc_index;
    try std.testing.expect(num_load_allocations > 1000);
    var fail_index: usize = 0;
    while (fail_index < num_load_allocations) : (fail_index += 997) {
        var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = fail_index });
        const scene = (RagdollScene.create(failing.allocator(), io, 2, 2, 0.4) catch continue).scene();
        defer scene.destroy();
        try std.testing.expectError(error.OutOfMemory, scene.load("Assets/"));
    }

    // StartTest and StopTest fail: the scene is loaded once, then the allocator fails from a later index on
    var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{});
    const scene = (try RagdollScene.create(failing.allocator(), io, 2, 2, 0.4)).scene();
    defer scene.destroy();
    try std.testing.expect(try scene.load("Assets/"));
    var num_start_stop_allocations: usize = 0;
    for (0..2) |_| { // The first run creates the shapes of the settings, which the later runs reuse
        const world = try TestPhysicsSystem.create(scene);
        defer world.destroy();
        const start = failing.alloc_index;
        try startAndStopForTest(scene, &world.system);
        num_start_stop_allocations = failing.alloc_index - start;
    }
    try std.testing.expect(num_start_stop_allocations > 0);
    for (0..num_start_stop_allocations) |i| {
        const world = try TestPhysicsSystem.create(scene);
        defer world.destroy();
        failing.fail_index = failing.alloc_index + i;
        try std.testing.expectError(error.OutOfMemory, startAndStopForTest(scene, &world.system));
        failing.fail_index = std.math.maxInt(usize);

        // A StopTest that failed keeps the ragdolls that are still in the system: a second StopTest removes them
        try scene.stopTest(&world.system);
    }

    // The scene still works
    const world = try TestPhysicsSystem.create(scene);
    defer world.destroy();
    try startAndStopForTest(scene, &world.system);
}

test "RagdollScene: destroy leaks the ragdolls that a failed StopTest left behind" {
    // Zolt test: StopTest runs out of memory and is not called again, like in main after an error. The physics system
    // is destroyed first, then the scene. Destroy must not release the ragdolls, because their bodies were in that
    // system, and it must not crash. An arena frees the leaked memory at the end.
    var arena: std.heap.ArenaAllocator = .init(std.testing.allocator);
    defer arena.deinit();
    var failing = std.testing.FailingAllocator.init(arena.allocator(), .{});
    const ragdoll_scene = try RagdollScene.create(failing.allocator(), std.testing.io, 2, 2, 0.4);
    const scene = ragdoll_scene.scene();
    defer scene.destroy();
    try std.testing.expect(try scene.load("Assets/"));
    const world = try TestPhysicsSystem.create(scene);
    defer world.destroy();
    try scene.startTest(&world.system, .linear_cast);

    // Every later allocation fails: StopTest cannot remove the first ragdoll, so all 2 x 2 x 2 ragdolls stay
    failing.fail_index = failing.alloc_index;
    try std.testing.expectError(error.OutOfMemory, scene.stopTest(&world.system));
    try std.testing.expectEqual(8, ragdoll_scene.ragdolls.items.len);
}

test {
    _ = LayersFile;
    _ = CharacterVirtualScene;
}
