//! Zolt helper of `zig build hello-world-check` (no Jolt counterpart): compares the stdout of the C++ HelloWorld with
//! the stdout of its Zig port (ZoltHelloWorld/HelloWorld.zig) byte by byte.
//!
//! Usage: CompareOutput <stdout of the C++ program> <stdout of the Zig program>
//!
//! Prints nothing and exits with 0 when the files are identical. Otherwise it prints the first differing line of both
//! programs to stderr and exits with 1 (the build step then fails and shows the message).

const std = @import("std");

pub fn main(init: std.process.Init) !u8 {
    const arena = init.arena.allocator();
    const io = init.io;

    const args = try init.minimal.args.toSlice(arena);
    if (args.len != 3) {
        std.debug.print("usage: {s} <stdout of the C++ program> <stdout of the Zig program>\n", .{if (args.len > 0) args[0] else "CompareOutput"});
        return 2;
    }

    const cwd = std.Io.Dir.cwd();
    const cpp_output = try cwd.readFileAlloc(io, args[1], arena, .unlimited);
    const zig_output = try cwd.readFileAlloc(io, args[2], arena, .unlimited);

    const first_difference = std.mem.indexOfDiff(u8, cpp_output, zig_output) orelse return 0;

    // Report the line that contains the first difference
    const line_start = if (std.mem.lastIndexOfScalar(u8, cpp_output[0..@min(first_difference, cpp_output.len)], '\n')) |i| i + 1 else 0;
    const line_number = std.mem.count(u8, cpp_output[0..line_start], "\n") + 1;
    std.debug.print(
        \\HelloWorld: the stdout of the Zig port differs from the C++ program at byte {d} (line {d}; C++: {d} bytes, {d} lines; Zig: {d} bytes, {d} lines)
        \\  C++: "{f}"
        \\  Zig: "{f}"
        \\
    , .{
        first_difference,
        line_number,
        cpp_output.len,
        std.mem.count(u8, cpp_output, "\n"),
        zig_output.len,
        std.mem.count(u8, zig_output, "\n"),
        std.zig.fmtString(lineAt(cpp_output, line_start)),
        std.zig.fmtString(lineAt(zig_output, line_start)),
    });
    return 1;
}

/// The line that starts at `start` (without its new line), empty past the end
fn lineAt(text: []const u8, start: usize) []const u8 {
    if (start >= text.len) return "";
    const end = std.mem.indexOfScalarPos(u8, text, start, '\n') orelse text.len;
    return text[start..end];
}
