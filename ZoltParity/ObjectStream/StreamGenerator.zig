//! Random object streams for the ObjectStream parity harness (ObjectStreamParity.zig, SerializationArchitecture.md
//! section 5): P3 (random streams), P4 (old schemas) and the streams of P5 (truncation).
//!
//! A document is generated from the C++ schema (ObjectStreamSchema.zig): the main object of a class, the objects that
//! its references create, the class declarations that the reader needs, and random values. Mutations then change the
//! document like an old or odd stream would (P4). The document is written as a text or a binary stream; both libraries
//! read the bytes.
//!
//! The rules of section 5 hold for every document:
//! - counts and string lengths always match the data that follows (Jolt allocates without bound);
//! - binary bools are 0 or 1, floats are finite and normal (or zero), also after the reader converts a dvec3 to a Vec3;
//!   enums have named values (the Zolt enum's, any value for bit flags and integers);
//! - references form no cycle (an object only references objects that were complete before it), and no reference
//!   points to the object that the reader takes as the main object (D4): the main object is never complete for the
//!   objects that mutations add, and finish() makes a reference to another first object null;
//! - no attribute is declared twice for one class (D11), no identifier is used twice (D2);
//! - an object other than the main object is never of a class derived from the main object's class, and finish() gives
//!   the request the class of the object that the reader takes as the main object when that class derives from it. So a
//!   read that succeeds always returns an object of the requested class (the C++ side writes it as that class);
//! - an attribute whose data is an identifier (a Ref) is never skipped: not by name, and not inside an object or a class
//!   by value that the reader skips (Jolt then reads class data: S15);
//! - nested arrays are never skipped: Jolt's SkipAttributeData reads the counts of all levels first and multiplies
//!   them, so it misreads a nested array (Zolt does the same); the misread lengths make Jolt allocate without bound;
//! - raw pointers stay null: an object that only a raw pointer holds leaks in both libraries.

const std = @import("std");
const zolt = @import("zolt");
const schema_mod = @import("ObjectStreamSchema.zig");

const Allocator = std.mem.Allocator;
const Schema = schema_mod.Schema;
const Class = schema_mod.Class;
const Attribute = schema_mod.Attribute;
const Element = schema_mod.Element;
const OSDataType = zolt.OSDataType;

pub const Format = enum { text, binary };

/// A value of a primitive type: the components in the order of the stream
pub const Primitive = union(enum) {
    uint8: u8,
    uint16: u16,
    int: i32,
    uint32: u32,
    uint64: u64,
    float: f32,
    double: f64,
    bool: bool,
    float3: [3]f32,
    float4: [4]f32,
    double3: [3]f64,
    vec3: [3]f32,
    dvec3: [3]f64,
    vec4: [4]f32,
    uvec4: [4]u32,
    quat: [4]f32,
    mat44: [16]f32,
    dmat44: struct { columns: [12]f32, translation: [3]f64 },
};

pub const Value = union(enum) {
    primitive: Primitive,
    string: []const u8,
    /// A reference or a raw pointer (0 is null)
    identifier: u32,
    /// A class by value: its class data
    instance: *Instance,
    /// An array: the count, then the elements
    array: []Value,
};

pub const Field = struct { key: []const u8, value: *Value };

/// The values of an object or of a class by value, by attribute key
pub const Instance = struct {
    class: []const u8,
    fields: std.ArrayList(Field) = .empty,

    pub fn get(self: *const Instance, key: []const u8) ?*Value {
        for (self.fields.items) |f| {
            if (std.mem.eql(u8, f.key, key)) return f.value;
        }
        return null;
    }
};

/// One attribute of a `declare` statement
pub const AttributeDecl = struct {
    /// The name in the stream
    name: []const u8,
    /// The field of the instances that holds the data
    key: []const u8,
    depth: u32,
    /// The element type: a primitive data type, `.instance` or `.pointer`
    data_type: OSDataType,
    class_name: []const u8,
    /// How the data is written (the reader needs the declaration of a class by value)
    element: Element,
    /// Doubles in the range of a float (a dvec3 that the reader converts to a Vec3)
    float_range: bool = false,
};

/// A `declare` statement. A class can have two (the second appends its attributes).
pub const Declaration = struct {
    class: []const u8,
    attributes: std.ArrayList(AttributeDecl) = .empty,
};

pub const Object = struct {
    class: []const u8,
    identifier: u32,
    instance: *Instance,
};

/// A slot that holds the identifier of a referenced object
pub const ReferenceSlot = struct {
    slot: *u32,
    /// The declared class of the reference (an index in the schema)
    class: usize,
};

pub const Document = struct {
    /// The class that both sides read the stream as
    request: [:0]const u8,
    /// The objects in stream order (the main object first, unless a mutation moves it)
    objects: std.ArrayList(*Object) = .empty,
    declarations: std.ArrayList(*Declaration) = .empty,
    instances: std.ArrayList(*Instance) = .empty,
    references: std.ArrayList(ReferenceSlot) = .empty,
    /// An object of an undeclared class before object `undeclared_at` (text only: the reader stops at its identifier)
    undeclared_at: ?usize = null,
};

pub const Generator = struct {
    arena: Allocator,
    schema: *const Schema,
    random: std.Random,
    doc: *Document,
    /// The class of the main object
    main: usize,
    main_object: *Object = undefined,
    /// Objects whose data is complete (they may be referenced, no cycle can form). Never the main object.
    complete: std.ArrayList(*Object) = .empty,
    identifiers: std.AutoHashMapUnmanaged(u32, void) = .empty,
    names: u32 = 0,

    pub const max_objects = 10;
    pub const max_depth = 4;

    /// A document with a main object of class `main` (creatable and ready)
    pub fn init(arena: Allocator, schema: *const Schema, random: std.Random, main: usize) !Generator {
        const doc = try arena.create(Document);
        doc.* = .{ .request = schema.classes[main].name };
        var self: Generator = .{ .arena = arena, .schema = schema, .random = random, .doc = doc, .main = main };
        self.main_object = try self.newObject(main, 0);
        // Objects that mutations add never reference the main object: Jolt deletes a main object that only an
        // unreferenced object references (D4). The main object is the last object that newObject completed.
        const last = self.complete.pop();
        std.debug.assert(last == self.main_object);

        // The main object first, the others in creation order or shuffled
        const rest = try arena.dupe(*Object, self.doc.objects.items[1..]);
        if (random.boolean()) random.shuffle(*Object, rest);
        self.doc.objects.clearRetainingCapacity();
        try self.doc.objects.append(arena, self.main_object);
        try self.doc.objects.appendSlice(arena, rest);

        for (self.doc.objects.items) |o| try self.ensureDeclaration(o.class);
        return self;
    }

    fn classIndex(self: *const Generator, name: []const u8) !usize {
        return self.schema.find(name) orelse error.HarnessError;
    }

    fn freshIdentifier(self: *Generator) !u32 {
        while (true) {
            const id = self.random.int(u32);
            if (id == 0) continue;
            const entry = try self.identifiers.getOrPut(self.arena, id);
            if (!entry.found_existing) return id;
        }
    }

    fn newObject(self: *Generator, class: usize, depth: u32) anyerror!*Object {
        const object = try self.arena.create(Object);
        object.* = .{ .class = self.schema.classes[class].name, .identifier = try self.freshIdentifier(), .instance = undefined };
        try self.doc.objects.append(self.arena, object);
        object.instance = try self.newInstance(class, depth);
        try self.complete.append(self.arena, object);
        return object;
    }

    fn newInstance(self: *Generator, class: usize, depth: u32) anyerror!*Instance {
        const c = &self.schema.classes[class];
        const instance = try self.arena.create(Instance);
        instance.* = .{ .class = c.name };
        try self.doc.instances.append(self.arena, instance);
        for (c.attributes) |*a| {
            const value = try self.arena.create(Value);
            try self.genValue(a, 0, depth, value);
            try instance.fields.append(self.arena, .{ .key = a.name, .value = value });
        }
        return instance;
    }

    fn genValue(self: *Generator, a: *const Attribute, level: u32, depth: u32, out: *Value) anyerror!void {
        if (level < a.depth) {
            const elements = try self.arena.alloc(Value, try self.pickCount(a));
            for (elements) |*e| try self.genValue(a, level + 1, depth, e);
            out.* = .{ .array = elements };
            return;
        }
        switch (a.element) {
            .primitive => out.* = if (a.data_type == .t_string) .{ .string = try randomString(self.arena, self.random) } else .{ .primitive = randomPrimitive(self.random, a.data_type, false) },
            .enumeration => out.* = .{ .primitive = .{ .uint32 = enumValue(self.random, a) } },
            // Raw pointers stay null: an object held only by a raw pointer leaks in both libraries
            .pointer => out.* = .{ .identifier = 0 },
            .reference => {
                out.* = .{ .identifier = try self.pickReference(try self.classIndex(a.class_name), depth) };
                try self.doc.references.append(self.arena, .{ .slot = &out.identifier, .class = try self.classIndex(a.class_name) });
            },
            .value => out.* = .{ .instance = try self.newInstance(try self.classIndex(a.class_name), depth) },
            .unknown => return error.HarnessError,
        }
    }

    /// A count that the array accepts (C++ probe): 0 to 3 elements, or the size of a C array
    fn pickCount(self: *Generator, a: *const Attribute) !usize {
        // The arrays of a Phase 9 attribute stay empty (PhysicsScene.mSoftBodies, until group S2)
        if (a.phase9) {
            if (a.counts & 1 == 0) return error.HarnessError;
            return 0;
        }
        const small = a.counts & 0xf;
        if (small != 0) {
            while (true) {
                const c = self.random.uintLessThan(u6, 4);
                if (small & (@as(u64, 1) << c) != 0) return c;
            }
        }
        if (a.counts == 0) return error.HarnessError;
        return @ctz(a.counts);
    }

    /// The classes that a reference to class `class` may point to: creatable and ready, never derived from the class of
    /// the main object (unless it is that class)
    fn referenceCandidate(self: *const Generator, candidate: usize, class: usize) bool {
        const c = &self.schema.classes[candidate];
        if (!c.creatable() or !c.ready()) return false;
        if (!self.schema.isKindOf(candidate, class)) return false;
        return candidate == self.main or !self.schema.isKindOf(candidate, self.main);
    }

    fn pickReference(self: *Generator, class: usize, depth: u32) !u32 {
        var candidates: std.ArrayList(usize) = .empty;
        for (self.schema.classes, 0..) |_, i| {
            if (self.referenceCandidate(i, class)) try candidates.append(self.arena, i);
        }
        const roll = self.random.uintLessThan(u32, 10);
        if (roll < 3 or candidates.items.len == 0) return 0;

        // An object that is already complete (a shared reference)
        if (roll < 6 or self.doc.objects.items.len >= max_objects or depth >= max_depth) {
            var existing: std.ArrayList(*Object) = .empty;
            for (self.complete.items) |o| {
                const i = try self.classIndex(o.class);
                if (self.referenceCandidate(i, class)) try existing.append(self.arena, o);
            }
            if (existing.items.len > 0) return existing.items[self.random.uintLessThan(usize, existing.items.len)].identifier;
            if (self.doc.objects.items.len >= max_objects or depth >= max_depth) return 0;
        }

        // A new object
        const object = try self.newObject(candidates.items[self.random.uintLessThan(usize, candidates.items.len)], depth + 1);
        return object.identifier;
    }

    fn findDeclaration(self: *const Generator, class: []const u8) ?*Declaration {
        for (self.doc.declarations.items) |d| {
            if (std.mem.eql(u8, d.class, class)) return d;
        }
        return null;
    }

    /// Declare `class` with its C++ attributes, and the classes by value that its attributes hold
    fn ensureDeclaration(self: *Generator, class: []const u8) !void {
        if (self.findDeclaration(class) != null) return;
        const c = &self.schema.classes[try self.classIndex(class)];
        const d = try self.arena.create(Declaration);
        d.* = .{ .class = c.name };
        try self.doc.declarations.append(self.arena, d);
        for (c.attributes) |a| {
            try d.attributes.append(self.arena, .{ .name = a.name, .key = a.name, .depth = a.depth, .data_type = a.data_type, .class_name = a.class_name, .element = a.element });
            if (a.element == .value) try self.ensureDeclaration(a.class_name);
        }
    }

    fn uniqueName(self: *Generator, prefix: []const u8) ![]const u8 {
        self.names += 1;
        return std.fmt.allocPrint(self.arena, "{s}{d}", .{ prefix, self.names });
    }

    // P4 mutations ------------------------------------------------------------------------------------------------------

    pub const Mutation = enum {
        /// An attribute that the stream does not declare keeps its default
        drop_attribute,
        reorder_attributes,
        /// An attribute of another name is skipped
        rename_attribute,
        /// An attribute of another type is skipped
        retype_attribute,
        /// vec3 is read into a DVec3 and dvec3 into a Vec3
        convert_vec3,
        /// A second `declare` appends its attributes (also unknown ones)
        declare_twice,
        /// Objects of a class that the factory does not know are skipped
        unknown_class,
        /// An object of an undeclared class ends the read (text only)
        undeclared_object,
        /// Objects of an abstract class are skipped
        abstract_object,
        /// An object with the null identifier is skipped
        null_identifier_object,
        /// The main object has the null identifier: the next object is the main object
        main_null_identifier,
        /// An object that nothing references is released (before the main object: it is the main object)
        extra_object,
        /// A reference to an identifier that no object has is null
        dangling_identifier,
        /// A reference to an object of the wrong class is null
        wrong_type_identifier,
        /// The main object is not of the requested class: the read fails
        wrong_main_class,
    };

    /// Apply `mutation`, false when it does not apply to this document
    pub fn mutate(self: *Generator, mutation: Mutation) !bool {
        const r = self.random;
        switch (mutation) {
            .drop_attribute => {
                const d = self.pickDeclaration(1) orelse return false;
                _ = d.attributes.orderedRemove(r.uintLessThan(usize, d.attributes.items.len));
            },
            .reorder_attributes => {
                const d = self.pickDeclaration(2) orelse return false;
                r.shuffle(AttributeDecl, d.attributes.items);
            },
            .rename_attribute => {
                const d = self.pickDeclaration(1) orelse return false;
                const a = &d.attributes.items[r.uintLessThan(usize, d.attributes.items.len)];
                if (a.element == .reference) return false; // Jolt skips a Ref as class data (S15)
                if (a.depth >= 2 or !self.skippable(a.class_name)) return false; // Jolt skips nested arrays wrongly
                a.name = try self.uniqueName("mZoltRenamed");
            },
            .retype_attribute => {
                const d = self.pickDeclaration(1) orelse return false;
                const a = &d.attributes.items[r.uintLessThan(usize, d.attributes.items.len)];
                const data_type = schema_mod.primitive_data_types[r.uintLessThan(usize, schema_mod.primitive_data_types.len)];
                // At most one array level: Jolt skips nested arrays wrongly
                const depth: u32 = if (r.uintLessThan(u32, 4) == 0) r.uintLessThan(u32, 2) else @min(a.depth, 1);
                if (data_type == a.data_type and depth == a.depth) return false;
                // The member that the reader matches by name. Its own type is no retype (an earlier mutation may have
                // changed the declaration), and an enum member would get a value without a name.
                const member = self.classAttribute(d.class, a.name);
                if (member) |m| {
                    if (data_type == m.data_type and depth == m.depth) return false;
                }
                // The reader converts a dvec3 to a Vec3 member and a vec3 to a DVec3 member (S11): doubles in the range
                // of a float, like convert_vec3
                const float_range = if (member) |m| depth == 0 and m.depth == 0 and isVec3Type(m.data_type) and isVec3Type(data_type) else false;
                const key = try self.uniqueName("$retyped");
                try self.addValues(d.class, key, depth, data_type, float_range);
                a.* = .{ .name = a.name, .key = key, .depth = depth, .data_type = data_type, .class_name = "", .element = .primitive, .float_range = float_range };
            },
            .convert_vec3 => {
                const Candidate = struct { declaration: *Declaration, attribute: *AttributeDecl };
                var candidates: std.ArrayList(Candidate) = .empty;
                for (self.doc.declarations.items) |d| {
                    for (d.attributes.items) |*a| {
                        if (a.depth == 0 and (a.data_type == .t_vec3 or a.data_type == .t_dvec3)) try candidates.append(self.arena, .{ .declaration = d, .attribute = a });
                    }
                }
                if (candidates.items.len == 0) return false;
                const candidate = candidates.items[r.uintLessThan(usize, candidates.items.len)];
                const a = candidate.attribute;
                const class = candidate.declaration.class;
                const data_type: OSDataType = if (a.data_type == .t_vec3) .t_dvec3 else .t_vec3;
                const key = try self.uniqueName("$converted");
                // Doubles in the range of a float: the reader converts them to a Vec3
                try self.addValues(class, key, 0, data_type, true);
                a.* = .{ .name = a.name, .key = key, .depth = 0, .data_type = data_type, .class_name = "", .element = .primitive, .float_range = true };
            },
            .declare_twice => {
                const d = self.pickDeclaration(2) orelse return false;
                const split = 1 + r.uintLessThan(usize, d.attributes.items.len - 1);
                const second = try self.arena.create(Declaration);
                second.* = .{ .class = d.class };
                try second.attributes.appendSlice(self.arena, d.attributes.items[split..]);
                d.attributes.shrinkRetainingCapacity(split);
                if (r.boolean()) {
                    const key = try self.uniqueName("mZoltExtra");
                    const data_type = schema_mod.primitive_data_types[r.uintLessThan(usize, schema_mod.primitive_data_types.len)];
                    try self.addValues(d.class, key, 0, data_type, false);
                    try second.attributes.insert(self.arena, r.uintLessThan(usize, second.attributes.items.len + 1), .{ .name = key, .key = key, .depth = 0, .data_type = data_type, .class_name = "", .element = .primitive });
                }
                const index = std.mem.indexOfScalar(*Declaration, self.doc.declarations.items, d).?;
                try self.doc.declarations.insert(self.arena, index + 1, second);
            },
            .unknown_class => {
                const name = try self.uniqueName("ZoltParityUnknown");
                const d = try self.arena.create(Declaration);
                d.* = .{ .class = name };
                for (0..1 + r.uintLessThan(usize, 4)) |_| {
                    const data_type = schema_mod.primitive_data_types[r.uintLessThan(usize, schema_mod.primitive_data_types.len)];
                    const key = try self.uniqueName("mValue");
                    try d.attributes.append(self.arena, .{ .name = key, .key = key, .depth = r.uintLessThan(u32, 2), .data_type = data_type, .class_name = "", .element = .primitive });
                }
                // A class by value: skipped through its declaration
                if (r.boolean()) {
                    if (self.pickReadyClass(.skipped)) |class| {
                        const key = try self.uniqueName("mInstance");
                        try d.attributes.append(self.arena, .{ .name = key, .key = key, .depth = 0, .data_type = .instance, .class_name = self.schema.classes[class].name, .element = .value });
                        try self.ensureDeclaration(self.schema.classes[class].name);
                    }
                }
                try self.doc.declarations.append(self.arena, d);
                for (0..1 + r.uintLessThan(usize, 2)) |_| {
                    const instance = try self.arena.create(Instance);
                    instance.* = .{ .class = name };
                    try self.doc.instances.append(self.arena, instance);
                    for (d.attributes.items) |a| {
                        const value = try self.arena.create(Value);
                        if (a.element == .value) {
                            value.* = .{ .instance = try self.newInstance(try self.classIndex(a.class_name), max_depth) };
                        } else try self.genSimple(a.depth, a.data_type, false, value);
                        try instance.fields.append(self.arena, .{ .key = a.key, .value = value });
                    }
                    try self.insertObject(.{ .class = name, .identifier = try self.freshIdentifier(), .instance = instance });
                }
            },
            .undeclared_object => self.doc.undeclared_at = r.uintLessThan(usize, self.doc.objects.items.len + 1),
            .abstract_object => {
                var abstract: std.ArrayList(usize) = .empty;
                for (self.schema.classes, 0..) |c, i| {
                    if (c.abstract and self.findDeclaration(c.name) == null) try abstract.append(self.arena, i);
                }
                if (abstract.items.len == 0) return false;
                const c = &self.schema.classes[abstract.items[r.uintLessThan(usize, abstract.items.len)]];
                const d = try self.arena.create(Declaration);
                d.* = .{ .class = c.name };
                const instance = try self.arena.create(Instance);
                instance.* = .{ .class = c.name };
                try self.doc.instances.append(self.arena, instance);
                // Its primitive attributes (the others are not probed for an abstract class)
                for (c.attributes) |*a| {
                    if (a.depth != 0 or a.data_type == .instance or a.data_type == .pointer) continue;
                    try d.attributes.append(self.arena, .{ .name = a.name, .key = a.name, .depth = 0, .data_type = a.data_type, .class_name = "", .element = .primitive });
                    const value = try self.arena.create(Value);
                    if (a.primitive == null) value.* = .{ .primitive = .{ .uint32 = enumValue(r, a) } } else try self.genSimple(0, a.data_type, false, value);
                    try instance.fields.append(self.arena, .{ .key = a.name, .value = value });
                }
                try self.doc.declarations.append(self.arena, d);
                try self.insertObject(.{ .class = c.name, .identifier = try self.freshIdentifier(), .instance = instance });
            },
            .null_identifier_object, .extra_object => {
                const class = self.pickReadyClass(if (mutation == .null_identifier_object) .skipped else .read) orelse return false;
                const object = try self.newObject(class, max_depth - 1);
                // newObject appended the object (and the objects it created) at the end: move it
                const index = std.mem.indexOfScalar(*Object, self.doc.objects.items, object).?;
                _ = self.doc.objects.orderedRemove(index);
                if (mutation == .null_identifier_object) object.identifier = 0;
                for (self.doc.objects.items) |o| try self.ensureDeclaration(o.class);
                try self.ensureDeclaration(object.class);
                try self.doc.objects.insert(self.arena, r.uintLessThan(usize, self.doc.objects.items.len + 1), object);
            },
            .main_null_identifier => {
                const main = self.main_object;
                if (main.identifier == 0 or !self.skippable(main.class)) return false;
                // The first other object that is created becomes the main object. finish() makes the references to it
                // null (D4).
                main.identifier = 0;
            },
            .dangling_identifier => {
                if (self.doc.references.items.len == 0) return false;
                const s = self.doc.references.items[r.uintLessThan(usize, self.doc.references.items.len)];
                s.slot.* = try self.freshIdentifier();
            },
            .wrong_type_identifier => {
                if (self.doc.references.items.len == 0) return false;
                const s = self.doc.references.items[r.uintLessThan(usize, self.doc.references.items.len)];
                var targets: std.ArrayList(*Object) = .empty;
                for (self.doc.objects.items) |o| {
                    const i = self.schema.find(o.class) orelse continue;
                    if (o.identifier != 0 and !self.schema.isKindOf(i, s.class)) try targets.append(self.arena, o);
                }
                if (targets.items.len == 0) return false;
                s.slot.* = targets.items[r.uintLessThan(usize, targets.items.len)].identifier;
            },
            .wrong_main_class => {
                var classes: std.ArrayList(usize) = .empty;
                for (self.schema.classes, 0..) |c, i| {
                    if (c.zolt != null and !self.schema.isKindOf(self.main, i)) try classes.append(self.arena, i);
                }
                if (classes.items.len == 0) return false;
                self.doc.request = self.schema.classes[classes.items[r.uintLessThan(usize, classes.items.len)]].name;
            },
        }
        return true;
    }

    /// After the mutations:
    /// - give every instance a value for each attribute that its class declares (an instance created after a retyped or
    ///   extra attribute was added has none yet);
    /// - a mutation can put another object first (extra_object, main_null_identifier). When the class of the object that
    ///   the reader takes as the main object derives from the requested class, request its class: the C++ side writes
    ///   the main object as the requested class, Jolt's sWriteObject would use its own class;
    /// - make every reference to that object null: Jolt deletes a main object that only an unreferenced object
    ///   references (D4).
    pub fn finish(self: *Generator) !void {
        for (self.doc.declarations.items) |d| {
            for (d.attributes.items) |a| {
                for (self.doc.instances.items) |instance| {
                    if (!std.mem.eql(u8, instance.class, d.class) or instance.get(a.key) != null) continue;
                    if (a.element != .primitive) return error.HarnessError;
                    const value = try self.arena.create(Value);
                    try self.genSimple(a.depth, a.data_type, a.float_range, value);
                    try instance.fields.append(self.arena, .{ .key = a.key, .value = value });
                }
            }
        }

        const main = self.readMainObject() orelse return;
        const class = try self.classIndex(main.class);
        const request = try self.classIndex(self.doc.request);
        if (class != request and self.schema.isKindOf(class, request)) self.doc.request = self.schema.classes[class].name;
        for (self.doc.references.items) |s| {
            if (s.slot.* == main.identifier) s.slot.* = 0;
        }
    }

    /// The object that the reader takes as the main object: the first object that it creates (ObjectStreamIn::Read). It
    /// skips objects with the null identifier, of an unknown class or of an abstract class, and stops at an object of an
    /// undeclared class.
    fn readMainObject(self: *const Generator) ?*Object {
        for (self.doc.objects.items, 0..) |o, index| {
            if (self.doc.undeclared_at == index) return null;
            if (o.identifier == 0) continue;
            const i = self.schema.find(o.class) orelse continue;
            if (self.schema.classes[i].abstract) continue;
            return o;
        }
        return null;
    }

    /// The attribute of `class` that the reader matches with the stream attribute `name` (the first one, S11), null for
    /// an unknown class or name
    fn classAttribute(self: *const Generator, class: []const u8, name: []const u8) ?*const Attribute {
        const index = self.schema.find(class) orelse return null;
        for (self.schema.classes[index].attributes) |*a| {
            if (std.mem.eql(u8, a.name, name)) return a;
        }
        return null;
    }

    fn pickDeclaration(self: *Generator, min_attributes: usize) ?*Declaration {
        var candidates: std.ArrayList(*Declaration) = .empty;
        for (self.doc.declarations.items) |d| {
            if (d.attributes.items.len >= min_attributes) candidates.append(self.arena, d) catch return null;
        }
        if (candidates.items.len == 0) return null;
        return candidates.items[self.random.uintLessThan(usize, candidates.items.len)];
    }

    /// A creatable and ready class that is not derived from the main class (unless it is the main class). The reader
    /// skips the data of a `.skipped` class, which must be skippable.
    fn pickReadyClass(self: *Generator, use: enum { read, skipped }) ?usize {
        var candidates: std.ArrayList(usize) = .empty;
        for (self.schema.classes, 0..) |c, i| {
            if (!c.creatable() or !c.ready() or (i != self.main and self.schema.isKindOf(i, self.main))) continue;
            if (use == .skipped and !self.skippable(c.name)) continue;
            candidates.append(self.arena, i) catch return null;
        }
        if (candidates.items.len == 0) return null;
        return candidates.items[self.random.uintLessThan(usize, candidates.items.len)];
    }

    /// True when the reader can skip the data of `class` (by value, transitively) without misreading the stream:
    /// - Jolt's SkipAttributeData reads every count of a nested array first and multiplies them, which misreads nested
    ///   arrays;
    /// - a Ref is declared `instance X` but its data is an identifier: skipping it reads class data of X (S15). With X
    ///   undeclared the skip stops before the identifier, and a binary stream then misreads the rest (D6).
    pub fn skippable(self: *const Generator, class: []const u8) bool {
        const index = self.schema.find(class) orelse return true; // A primitive name, an unknown class
        for (self.schema.classes[index].attributes) |a| {
            if (a.depth >= 2) return false;
            // An attribute that was not probed (of an abstract class) can be a Ref too
            if (a.element == .reference or a.element == .unknown) return false;
            if (a.element == .value and !std.mem.eql(u8, a.class_name, class) and !self.skippable(a.class_name)) return false;
        }
        return true;
    }

    fn insertObject(self: *Generator, object: Object) !void {
        const o = try self.arena.create(Object);
        o.* = object;
        try self.doc.objects.insert(self.arena, self.random.uintLessThan(usize, self.doc.objects.items.len + 1), o);
    }

    /// Add a value under `key` to every instance of `class` (a retyped or extra attribute)
    fn addValues(self: *Generator, class: []const u8, key: []const u8, depth: u32, data_type: OSDataType, float_range: bool) !void {
        for (self.doc.instances.items) |instance| {
            if (!std.mem.eql(u8, instance.class, class)) continue;
            const value = try self.arena.create(Value);
            try self.genSimple(depth, data_type, float_range, value);
            try instance.fields.append(self.arena, .{ .key = key, .value = value });
        }
    }

    /// A value of `depth` nested arrays (0 to 3 elements) of a primitive
    fn genSimple(self: *Generator, depth: u32, data_type: OSDataType, float_range: bool, out: *Value) !void {
        if (depth > 0) {
            const elements = try self.arena.alloc(Value, self.random.uintLessThan(usize, 4));
            for (elements) |*e| try self.genSimple(depth - 1, data_type, float_range, e);
            out.* = .{ .array = elements };
            return;
        }
        out.* = if (data_type == .t_string) .{ .string = try randomString(self.arena, self.random) } else .{ .primitive = randomPrimitive(self.random, data_type, float_range) };
    }
};

fn isVec3Type(data_type: OSDataType) bool {
    return data_type == .t_vec3 or data_type == .t_dvec3;
}

fn enumValue(r: std.Random, a: *const Attribute) u32 {
    const e = a.enum_values orelse return r.uintLessThan(u32, 2);
    if (e.values.len > 0) return e.values[r.uintLessThan(usize, e.values.len)];
    const value = r.int(u32);
    return if (e.bits >= 32) value else value & ((@as(u32, 1) << @intCast(e.bits)) - 1);
}

/// A finite float that is zero or normal (a subnormal cannot be read from a text stream)
pub fn randomFloat(r: std.Random) f32 {
    const specials = [_]f32{ 0, -0.0, 1, -1, 0.5, 2, 0.1, -3.14159274, 1.0e-30, 1.0e30, std.math.floatMin(f32), -std.math.floatMin(f32), std.math.floatMax(f32), -std.math.floatMax(f32) };
    if (r.uintLessThan(u32, 5) == 0) return specials[r.uintLessThan(usize, specials.len)];
    const mantissa = 1 + r.float(f32);
    const exponent = r.intRangeAtMost(i32, -60, 60);
    const value = std.math.ldexp(mantissa, exponent);
    return if (r.boolean()) -value else value;
}

/// A finite double that is zero or normal; `float_range`: also inside the range of a float
pub fn randomDouble(r: std.Random, float_range: bool) f64 {
    if (float_range) return randomFloatAsDouble(r);
    const specials = [_]f64{ 0, -0.0, 1, -1, 0.1, std.math.floatEps(f64), std.math.floatMin(f64), std.math.floatMax(f64), -std.math.floatMax(f64), 1.0e-300, 1.0e300 };
    if (r.uintLessThan(u32, 5) == 0) return specials[r.uintLessThan(usize, specials.len)];
    const mantissa = 1 + r.float(f64);
    const exponent = r.intRangeAtMost(i32, -200, 200);
    const value = std.math.ldexp(mantissa, exponent);
    return if (r.boolean()) -value else value;
}

fn randomFloatAsDouble(r: std.Random) f64 {
    const mantissa = 1 + r.float(f64);
    const value = std.math.ldexp(mantissa, r.intRangeAtMost(i32, -60, 60));
    return if (r.boolean()) -value else value;
}

fn randomInt(r: std.Random, comptime T: type) T {
    const specials = [_]T{ 0, 1, std.math.maxInt(T), std.math.minInt(T) };
    if (r.uintLessThan(u32, 4) == 0) return specials[r.uintLessThan(usize, specials.len)];
    // A small value (below 256 for a uint8)
    const small: u32 = @min(1000, std.math.maxInt(T));
    return if (r.boolean()) r.int(T) else @intCast(r.uintLessThan(u32, small));
}

pub fn randomPrimitive(r: std.Random, data_type: OSDataType, float_range: bool) Primitive {
    var f: [16]f32 = undefined;
    for (&f) |*x| x.* = randomFloat(r);
    var d: [3]f64 = undefined;
    for (&d) |*x| x.* = randomDouble(r, float_range);
    return switch (data_type) {
        .t_uint8 => .{ .uint8 = randomInt(r, u8) },
        .t_uint16 => .{ .uint16 = randomInt(r, u16) },
        .t_int => .{ .int = randomInt(r, i32) },
        .t_uint32 => .{ .uint32 = randomInt(r, u32) },
        .t_uint64 => .{ .uint64 = randomInt(r, u64) },
        .t_float => .{ .float = f[0] },
        .t_double => .{ .double = d[0] },
        .t_bool => .{ .bool = r.boolean() },
        .t_float3 => .{ .float3 = f[0..3].* },
        .t_float4 => .{ .float4 = f[0..4].* },
        .t_double3 => .{ .double3 = d },
        .t_vec3 => .{ .vec3 = f[0..3].* },
        .t_dvec3 => .{ .dvec3 = d },
        .t_vec4 => .{ .vec4 = f[0..4].* },
        .t_uvec4 => .{ .uvec4 = .{ randomInt(r, u32), randomInt(r, u32), randomInt(r, u32), randomInt(r, u32) } },
        .t_quat => .{ .quat = f[0..4].* },
        .t_mat44 => .{ .mat44 = f },
        .t_dmat44 => .{ .dmat44 = .{ .columns = f[0..12].*, .translation = d } },
        else => unreachable,
    };
}

/// A string with the characters that the text format escapes (backslash, quote, newline, tab), spaces and high bytes
pub fn randomString(arena: Allocator, r: std.Random) ![]const u8 {
    const alphabet = "abcXYZ019 _-.\\\"\n\t\r\xe9\xff";
    const length = if (r.uintLessThan(u32, 4) == 0) 0 else r.uintLessThan(usize, 13);
    const s = try arena.alloc(u8, length);
    for (s) |*c| c.* = if (r.uintLessThan(u32, 50) == 0) 0 else alphabet[r.uintLessThan(usize, alphabet.len)];
    return s;
}

// Writing a document ------------------------------------------------------------------------------------------------

/// Write `doc` as a stream of `format`. `random` varies the layout (white space, string table use, identifier form).
pub fn write(arena: Allocator, doc: *const Document, format: Format, random: std.Random) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(arena);
    var w: StreamWriter = .{ .arena = arena, .writer = &out.writer, .format = format, .random = random };
    try w.writeDocument(doc);
    return out.toOwnedSlice();
}

const StreamWriter = struct {
    arena: Allocator,
    writer: *std.Io.Writer,
    format: Format,
    random: std.Random,
    /// Binary: the string table (string to its first identifier)
    strings: std.StringHashMapUnmanaged(u32) = .empty,
    next_string: u32 = 0x80000000,
    /// The attributes of each class as the stream has declared them so far
    descriptions: std.StringHashMapUnmanaged(std.ArrayList(*const AttributeDecl)) = .empty,

    fn writeDocument(self: *StreamWriter, doc: *const Document) !void {
        try self.writer.writeAll(if (self.format == .text) "TOS 1.00" else "BOS 1.00");

        // Every declaration a class needs comes before its first object (all parts of a class together)
        var emitted: std.StringHashMapUnmanaged(void) = .empty;
        for (doc.objects.items, 0..) |o, index| {
            if (doc.undeclared_at == index) try self.writeUndeclared();
            var needed: std.ArrayList([]const u8) = .empty;
            try self.needs(doc, o.class, &needed);
            if (self.random.boolean()) self.random.shuffle([]const u8, needed.items);
            for (needed.items) |class| {
                if (emitted.contains(class)) continue;
                try emitted.put(self.arena, class, {});
                for (doc.declarations.items) |d| {
                    if (std.mem.eql(u8, d.class, class)) try self.writeDeclaration(d);
                }
            }
            try self.writeObject(o);
        }
        if (doc.undeclared_at == doc.objects.items.len) try self.writeUndeclared();

        // Declarations that no object needed
        for (doc.declarations.items) |d| {
            if (!emitted.contains(d.class)) try self.writeDeclaration(d);
        }
    }

    /// The class and the classes by value of its declared attributes (transitively)
    fn needs(self: *StreamWriter, doc: *const Document, class: []const u8, list: *std.ArrayList([]const u8)) !void {
        for (list.items) |c| {
            if (std.mem.eql(u8, c, class)) return;
        }
        try list.append(self.arena, class);
        for (doc.declarations.items) |d| {
            if (!std.mem.eql(u8, d.class, class)) continue;
            for (d.attributes.items) |a| {
                if (a.element == .value) try self.needs(doc, a.class_name, list);
            }
        }
    }

    fn separator(self: *StreamWriter) !void {
        if (self.format != .text) return;
        const choices = [_][]const u8{ " ", " ", " ", "\r\n", "\n", "\t", "  ", "\r\n    " };
        try self.writer.writeAll(choices[self.random.uintLessThan(usize, choices.len)]);
    }

    fn writeDataType(self: *StreamWriter, data_type: OSDataType) !void {
        if (self.format == .binary) return self.writer.writeInt(u32, @intFromEnum(data_type), .little);
        try self.separator();
        const word = schema_mod.wordOfDataType(data_type);
        // Data type words are case insensitive
        if (self.random.uintLessThan(u32, 10) == 0) {
            for (word) |c| try self.writer.writeByte(std.ascii.toUpper(c));
        } else try self.writer.writeAll(word);
    }

    fn writeName(self: *StreamWriter, name: []const u8) !void {
        if (self.format == .binary) return self.writeBinaryString(name);
        try self.separator();
        try self.writer.writeAll(name);
    }

    fn writeIdentifier(self: *StreamWriter, identifier: u32) !void {
        if (self.format == .binary) return self.writer.writeInt(u32, identifier, .little);
        try self.separator();
        // strtoul(word, 16): Jolt writes %08X, other forms read the same
        switch (self.random.uintLessThan(u32, 8)) {
            0 => try self.writer.print("{x:0>8}", .{identifier}),
            1 => try self.writer.print("{X}", .{identifier}),
            2 => try self.writer.print("0x{X}", .{identifier}),
            else => try self.writer.print("{X:0>8}", .{identifier}),
        }
    }

    fn writeCount(self: *StreamWriter, count: usize) !void {
        if (self.format == .binary) return self.writer.writeInt(u32, @intCast(count), .little);
        try self.separator();
        try self.writer.print("{d}", .{count});
    }

    fn writeBinaryString(self: *StreamWriter, s: []const u8) !void {
        if (s.len == 0) return self.writer.writeInt(u32, 0, .little);
        if (self.strings.get(s)) |id| {
            if (self.random.uintLessThan(u32, 5) != 0) return self.writer.writeInt(u32, id, .little);
        }
        try self.writer.writeInt(u32, @intCast(s.len), .little);
        try self.writer.writeAll(s);
        const entry = try self.strings.getOrPut(self.arena, s);
        if (!entry.found_existing) entry.value_ptr.* = self.next_string;
        self.next_string += 1;
    }

    fn writeDeclaration(self: *StreamWriter, d: *const Declaration) !void {
        try self.writeDataType(.declare);
        try self.writeName(d.class);
        try self.writeCount(d.attributes.items.len);
        const entry = try self.descriptions.getOrPut(self.arena, d.class);
        if (!entry.found_existing) entry.value_ptr.* = .empty;
        for (d.attributes.items) |*a| {
            try self.writeName(a.name);
            for (0..a.depth) |_| try self.writeDataType(.array);
            try self.writeDataType(a.data_type);
            if (a.data_type == .instance or a.data_type == .pointer) try self.writeName(a.class_name);
            try entry.value_ptr.append(self.arena, a);
        }
    }

    fn writeObject(self: *StreamWriter, o: *const Object) !void {
        try self.writeDataType(.object);
        try self.writeName(o.class);
        try self.writeIdentifier(o.identifier);
        try self.writeInstance(o.instance);
    }

    /// An object of a class that the stream does not declare: the reader does not read its identifier, the next word is
    /// not a data type and the read ends (text only)
    fn writeUndeclared(self: *StreamWriter) !void {
        try self.writeDataType(.object);
        try self.writeName("ZoltParityUndeclared");
        try self.writeIdentifier(1);
    }

    /// The class data of `instance` as the stream describes its class so far
    fn writeInstance(self: *StreamWriter, instance: *const Instance) anyerror!void {
        const description = self.descriptions.get(instance.class) orelse return error.HarnessError;
        for (description.items) |a| {
            const value = instance.get(a.key) orelse return error.HarnessError;
            try self.writeValue(value);
        }
    }

    fn writeValue(self: *StreamWriter, value: *const Value) anyerror!void {
        switch (value.*) {
            .array => |elements| {
                try self.writeCount(elements.len);
                for (elements) |*e| try self.writeValue(e);
            },
            .primitive => |p| try self.writePrimitive(p),
            .string => |s| try self.writeString(s),
            .identifier => |id| try self.writeIdentifier(id),
            .instance => |i| try self.writeInstance(i),
        }
    }

    fn writeString(self: *StreamWriter, s: []const u8) !void {
        if (self.format == .binary) return self.writeBinaryString(s);
        try self.separator();
        try self.writer.writeByte('"');
        for (s) |c| {
            const r = self.random.uintLessThan(u32, 4);
            switch (c) {
                '\\' => try self.writer.writeAll("\\\\"),
                '"' => try self.writer.writeAll("\\\""),
                // An escape, or the raw character (which the reader drops)
                '\n' => try self.writer.writeAll(if (r == 0) "\n" else "\\n"),
                '\t' => try self.writer.writeAll(if (r == 0) "\t" else "\\t"),
                else => {
                    // Any other escaped character is dropped by the reader
                    if (r == 0 and self.random.uintLessThan(u32, 4) == 0) try self.writer.writeByte('\\');
                    try self.writer.writeByte(c);
                },
            }
        }
        try self.writer.writeByte('"');
    }

    fn writePrimitive(self: *StreamWriter, p: Primitive) !void {
        switch (p) {
            .uint8 => |v| try self.writeInt(u8, v),
            .uint16 => |v| try self.writeInt(u16, v),
            .int => |v| try self.writeInt(i32, v),
            .uint32 => |v| try self.writeInt(u32, v),
            .uint64 => |v| try self.writeInt(u64, v),
            .float => |v| try self.writeFloat(f32, v),
            .double => |v| try self.writeFloat(f64, v),
            .bool => |v| {
                if (self.format == .binary) return self.writer.writeByte(@intFromBool(v));
                try self.separator();
                const word = if (v) "true" else "false";
                // Bools are case insensitive
                if (self.random.uintLessThan(u32, 8) == 0) {
                    for (word) |c| try self.writer.writeByte(std.ascii.toUpper(c));
                } else try self.writer.writeAll(word);
            },
            .uvec4 => |v| for (v) |x| try self.writeInt(u32, x),
            .double3, .dvec3 => |v| for (v) |x| try self.writeFloat(f64, x),
            .dmat44 => |v| {
                for (v.columns) |x| try self.writeFloat(f32, x);
                for (v.translation) |x| try self.writeFloat(f64, x);
            },
            inline .float3, .float4, .vec3, .vec4, .quat, .mat44 => |v| for (v) |x| try self.writeFloat(f32, x),
        }
    }

    fn writeInt(self: *StreamWriter, comptime T: type, value: T) !void {
        if (self.format == .binary) return self.writer.writeInt(T, value, .little);
        try self.separator();
        try self.writer.print("{d}", .{value});
    }

    fn writeFloat(self: *StreamWriter, comptime T: type, value: T) !void {
        if (self.format == .binary) return self.writer.writeInt(std.meta.Int(.unsigned, @bitSizeOf(T)), @bitCast(value), .little);
        try self.separator();
        // The shortest form that reads back exactly, in scientific or decimal notation
        if (self.random.boolean() and @abs(value) < 1.0e6 and (value == 0 or @abs(value) > 1.0e-6)) {
            try self.writer.print("{d}", .{value});
        } else try self.writer.print("{e}", .{value});
    }
};

// Tests of the generator rules: hand made schemas, no C++ side ------------------------------------------------------

const TestClass = struct {
    name: [:0]const u8,
    bases: []const []const u8 = &.{},
    attributes: []const Attribute = &.{},
};

/// A schema of creatable classes that Zolt declares. IsKindOf follows `bases`.
fn testSchema(arena: Allocator, classes: []const TestClass) !Schema {
    const n = classes.len;
    const list = try arena.alloc(Class, n);
    for (classes, list) |t, *c| c.* = .{
        .name = t.name,
        .line = "",
        .hash = 0,
        .abstract = false,
        .bases = t.bases,
        .attributes = try arena.dupe(Attribute, t.attributes),
        .outside = false,
        .primitive = false,
        .zolt = zolt.RTTI.of(zolt.SpringSettings), // Any RTTI: the class is declared and ready
    };
    const kind_of = try arena.alloc(bool, n * n);
    for (0..n) |i| {
        for (0..n) |j| kind_of[i * n + j] = testDerives(classes, i, classes[j].name);
    }
    return .{ .classes = list, .kind_of = kind_of };
}

fn testDerives(classes: []const TestClass, index: usize, base: []const u8) bool {
    if (std.mem.eql(u8, classes[index].name, base)) return true;
    for (classes[index].bases) |b| {
        for (classes, 0..) |c, j| {
            if (std.mem.eql(u8, c.name, b) and testDerives(classes, j, base)) return true;
        }
    }
    return false;
}

fn testAttribute(name: []const u8, data_type: OSDataType, element: Element, class_name: []const u8) Attribute {
    return .{ .name = name, .type_text = "", .primitive = if (class_name.len > 0) class_name else "", .depth = 0, .data_type = data_type, .class_name = class_name, .element = element };
}

/// The object that the reader takes as the main object: the first object that it creates (ObjectStreamIn::Read). A copy
/// of Generator.readMainObject, so that the tests do not rely on the code that they check.
fn testReadMain(schema: *const Schema, doc: *const Document) ?*Object {
    for (doc.objects.items, 0..) |o, index| {
        if (doc.undeclared_at == index) return null;
        if (o.identifier == 0) continue;
        const i = schema.find(o.class) orelse continue;
        if (schema.classes[i].abstract) continue;
        return o;
    }
    return null;
}

test "ObjectStream harness: random values of every primitive type (floats finite and normal or zero)" {
    for (0..2000) |seed| {
        var prng: std.Random.DefaultPrng = .init(seed);
        for (schema_mod.primitive_data_types) |data_type| {
            if (data_type == .t_string) continue;
            switch (randomPrimitive(prng.random(), data_type, false)) {
                .float => |x| try std.testing.expect(x == 0 or std.math.isNormal(x)),
                .double => |x| try std.testing.expect(x == 0 or std.math.isNormal(x)),
                inline .float3, .float4, .vec3, .vec4, .quat, .mat44, .double3, .dvec3 => |v| for (v) |x| try std.testing.expect(x == 0 or std.math.isNormal(x)),
                .dmat44 => |v| {
                    for (v.columns) |x| try std.testing.expect(x == 0 or std.math.isNormal(x));
                    for (v.translation) |x| try std.testing.expect(x == 0 or std.math.isNormal(x));
                },
                else => {},
            }
        }
    }
}

test "ObjectStream harness: the generator never skips a Ref, also not inside a skipped object or class by value" {
    var arena_state: std.heap.ArenaAllocator = .init(std.testing.allocator);
    defer arena_state.deinit();
    const arena = arena_state.allocator();
    const schema = try testSchema(arena, &.{
        .{ .name = "Target", .attributes = &.{testAttribute("mValue", .t_float, .primitive, "")} },
        .{ .name = "Holder", .attributes = &.{ testAttribute("mValue", .t_float, .primitive, ""), testAttribute("mTarget", .instance, .reference, "Target") } },
        .{ .name = "Outer", .attributes = &.{testAttribute("mHolder", .instance, .value, "Holder")} },
        .{ .name = "Plain", .attributes = &.{ testAttribute("mValue", .t_float, .primitive, ""), testAttribute("mTarget", .instance, .value, "Target") } },
    });
    var prng: std.Random.DefaultPrng = .init(1);
    const g = try Generator.init(arena, &schema, prng.random(), 3);
    try std.testing.expect(g.skippable("Target"));
    try std.testing.expect(g.skippable("Plain"));
    // A Ref is declared `instance Target` but its data is an identifier: skipping it reads class data (S15)
    try std.testing.expect(!g.skippable("Holder"));
    try std.testing.expect(!g.skippable("Outer"));
}

test "ObjectStream harness: a vec3 member retyped to dvec3 gets doubles in the range of a float (the reader converts them)" {
    var arena_state: std.heap.ArenaAllocator = .init(std.testing.allocator);
    defer arena_state.deinit();
    const arena = arena_state.allocator();
    const schema = try testSchema(arena, &.{
        .{ .name = "Body", .attributes = &.{ testAttribute("mPosition", .t_vec3, .primitive, ""), testAttribute("mRadius", .t_float, .primitive, "") } },
    });
    var converted: usize = 0;
    for (0..400) |seed| {
        var prng: std.Random.DefaultPrng = .init(seed);
        var g = try Generator.init(arena, &schema, prng.random(), 0);
        if (!try g.mutate(.retype_attribute)) continue;
        try g.finish();
        for (g.doc.declarations.items) |d| {
            for (d.attributes.items) |a| {
                if (!std.mem.eql(u8, a.name, "mPosition") or a.depth != 0 or a.data_type != .t_dvec3) continue;
                converted += 1;
                try std.testing.expect(a.float_range);
                for (g.doc.instances.items) |instance| {
                    const value = instance.get(a.key) orelse continue;
                    for (value.primitive.dvec3) |x| {
                        const f: f32 = @floatCast(x);
                        try std.testing.expect(std.math.isFinite(f) and (f == 0 or std.math.isNormal(f)));
                    }
                }
            }
        }
    }
    try std.testing.expect(converted > 0);
}

test "ObjectStream harness: no reference points to the object that the reader takes as the main object (D4)" {
    var arena_state: std.heap.ArenaAllocator = .init(std.testing.allocator);
    defer arena_state.deinit();
    const arena = arena_state.allocator();
    const schema = try testSchema(arena, &.{
        .{ .name = "Node", .attributes = &.{ testAttribute("mValue", .t_float, .primitive, ""), testAttribute("mChild", .instance, .reference, "Node") } },
        .{ .name = "Leaf", .attributes = &.{testAttribute("mNode", .instance, .reference, "Node")} },
    });
    const mutations = [_]Generator.Mutation{ .extra_object, .null_identifier_object, .main_null_identifier, .wrong_type_identifier };
    var checked: usize = 0;
    for (0..300) |seed| {
        var prng: std.Random.DefaultPrng = .init(seed);
        const r = prng.random();
        var g = try Generator.init(arena, &schema, r, 0);
        for (0..1 + r.uintLessThan(usize, 3)) |_| _ = try g.mutate(mutations[r.uintLessThan(usize, mutations.len)]);
        try g.finish();
        const main = testReadMain(&schema, g.doc) orelse continue;
        checked += 1;
        for (g.doc.references.items) |s| try std.testing.expect(s.slot.* != main.identifier);
    }
    try std.testing.expect(checked > 0);
}

test "ObjectStream harness: the object that the reader takes as the main object is of the requested class, or not kind of it" {
    var arena_state: std.heap.ArenaAllocator = .init(std.testing.allocator);
    defer arena_state.deinit();
    const arena = arena_state.allocator();
    const schema = try testSchema(arena, &.{
        .{ .name = "A", .attributes = &.{testAttribute("mValue", .t_float, .primitive, "")} },
        .{ .name = "B", .attributes = &.{testAttribute("mValue", .t_float, .primitive, "")} },
        .{ .name = "C", .bases = &.{"B"}, .attributes = &.{ testAttribute("mValue", .t_float, .primitive, ""), testAttribute("mExtra", .t_float, .primitive, "") } },
    });
    const mutations = [_]Generator.Mutation{ .wrong_main_class, .extra_object, .main_null_identifier };
    var checked: usize = 0;
    for (0..300) |seed| {
        var prng: std.Random.DefaultPrng = .init(seed);
        const r = prng.random();
        var g = try Generator.init(arena, &schema, r, 0);
        for (0..2 + r.uintLessThan(usize, 2)) |_| _ = try g.mutate(mutations[r.uintLessThan(usize, mutations.len)]);
        try g.finish();
        const main = testReadMain(&schema, g.doc) orelse continue;
        // Not kind of the request: the read fails on both sides
        if (!schema.isKindOf(schema.find(main.class).?, schema.find(g.doc.request).?)) continue;
        checked += 1;
        try std.testing.expectEqualStrings(g.doc.request, main.class);
    }
    try std.testing.expect(checked > 0);
}
