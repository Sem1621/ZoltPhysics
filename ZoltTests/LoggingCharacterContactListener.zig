//! Port of: UnitTests/LoggingCharacterContactListener.h
//! Status: complete
//!
//! Character contact listener that just logs the calls made to it for later validation. Register it with
//! `character.setListener(.init(&listener))`, or forward the callbacks of another listener to it (CharacterVirtualTests).
//! - The log is an ArrayList with an explicit allocator: a failed append is remembered and `checkError` reports it as
//!   error.OutOfMemory after the update (Jolt's push_back cannot fail, a callback cannot return an error).
//! - CharacterVirtual calls its listener on the thread that updates the character, so there is no mutex (like Jolt).
//! - Jolt's four `Find` / `Contains` overloads are `find` / `contains` (character vs body), `findCharacter` /
//!   `containsCharacter` (character vs character), `findSubShape` / `containsSubShape` (character vs body and sub
//!   shape) and `findCharacterSubShape` / `containsCharacterSubShape` (character vs character and sub shape).

const std = @import("std");
const zolt = @import("zolt");

const Allocator = std.mem.Allocator;
const BodyID = zolt.BodyID;
const SubShapeID = zolt.SubShapeID;
const CharacterID = zolt.CharacterID;
const CharacterVirtual = zolt.CharacterVirtual;
const CharacterContact = zolt.CharacterContact;
const CharacterContactSettings = zolt.CharacterContactSettings;

/// Character contact listener that just logs the calls made to it for later validation
pub const LoggingCharacterContactListener = struct {
    /// Contact callback type
    pub const Type = enum {
        validate_body,
        validate_character,
        add_body,
        persist_body,
        remove_body,
        add_character,
        persist_character,
        remove_character,
    };

    /// Entry written when a contact callback happens
    pub const LogEntry = struct {
        type: Type,
        character: *const CharacterVirtual,
        body2: BodyID,
        character_id2: CharacterID,
        sub_shape_id2: SubShapeID,
    };

    allocator: Allocator,
    log: std.ArrayList(LogEntry) = .empty,
    /// Zolt: an append failed (out of memory)
    out_of_memory: bool = false,

    pub fn init(allocator: Allocator) LoggingCharacterContactListener {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *LoggingCharacterContactListener) void {
        self.log.deinit(self.allocator);
    }

    fn append(self: *LoggingCharacterContactListener, entry: LogEntry) void {
        self.log.append(self.allocator, entry) catch {
            self.out_of_memory = true;
        };
    }

    pub fn onContactValidate(self: *LoggingCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact) bool {
        self.append(.{ .type = .validate_body, .character = character, .body2 = contact.base.body_b, .character_id2 = .invalid, .sub_shape_id2 = contact.base.sub_shape_id_b });
        return true;
    }

    pub fn onCharacterContactValidate(self: *LoggingCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact) bool {
        self.append(.{ .type = .validate_character, .character = character, .body2 = .invalid, .character_id2 = contact.character_b.?.getID(), .sub_shape_id2 = contact.base.sub_shape_id_b });
        return true;
    }

    pub fn onContactAdded(self: *LoggingCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        _ = settings;
        self.append(.{ .type = .add_body, .character = character, .body2 = contact.base.body_b, .character_id2 = .invalid, .sub_shape_id2 = contact.base.sub_shape_id_b });
    }

    pub fn onContactPersisted(self: *LoggingCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        _ = settings;
        self.append(.{ .type = .persist_body, .character = character, .body2 = contact.base.body_b, .character_id2 = .invalid, .sub_shape_id2 = contact.base.sub_shape_id_b });
    }

    pub fn onContactRemoved(self: *LoggingCharacterContactListener, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID) void {
        self.append(.{ .type = .remove_body, .character = character, .body2 = body_id2, .character_id2 = .invalid, .sub_shape_id2 = sub_shape_id2 });
    }

    pub fn onCharacterContactAdded(self: *LoggingCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        _ = settings;
        self.append(.{ .type = .add_character, .character = character, .body2 = .invalid, .character_id2 = contact.character_b.?.getID(), .sub_shape_id2 = contact.base.sub_shape_id_b });
    }

    pub fn onCharacterContactPersisted(self: *LoggingCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        _ = settings;
        self.append(.{ .type = .persist_character, .character = character, .body2 = .invalid, .character_id2 = contact.character_b.?.getID(), .sub_shape_id2 = contact.base.sub_shape_id_b });
    }

    pub fn onCharacterContactRemoved(self: *LoggingCharacterContactListener, character: *const CharacterVirtual, other_character_id: CharacterID, sub_shape_id2: SubShapeID) void {
        self.append(.{ .type = .remove_character, .character = character, .body2 = .invalid, .character_id2 = other_character_id, .sub_shape_id2 = sub_shape_id2 });
    }

    /// Zolt: report a failed append (error.OutOfMemory)
    pub fn checkError(self: *const LoggingCharacterContactListener) !void {
        if (self.out_of_memory) return error.OutOfMemory;
    }

    pub fn clear(self: *LoggingCharacterContactListener) void {
        self.log.clearRetainingCapacity();
    }

    pub fn getEntryCount(self: *const LoggingCharacterContactListener) usize {
        return self.log.items.len;
    }

    pub fn getEntry(self: *const LoggingCharacterContactListener, idx: usize) *const LogEntry {
        return &self.log.items[idx];
    }

    /// Find first event with a particular type and involving a particular character vs body
    pub fn find(self: *const LoggingCharacterContactListener, entry_type: Type, character: *const CharacterVirtual, body2: BodyID) ?usize {
        for (self.log.items, 0..) |e, i| {
            if (e.type == entry_type and e.character == character and e.body2.eql(body2) and e.character_id2.isInvalid())
                return i;
        }

        return null;
    }

    /// Check if event with a particular type and involving a particular character vs body
    pub fn contains(self: *const LoggingCharacterContactListener, entry_type: Type, character: *const CharacterVirtual, body2: BodyID) bool {
        return self.find(entry_type, character, body2) != null;
    }

    /// Find first event with a particular type and involving a particular character vs character
    pub fn findCharacter(self: *const LoggingCharacterContactListener, entry_type: Type, character: *const CharacterVirtual, other_character_id: CharacterID) ?usize {
        for (self.log.items, 0..) |e, i| {
            if (e.type == entry_type and e.character == character and e.body2.isInvalid() and e.character_id2.eql(other_character_id))
                return i;
        }

        return null;
    }

    /// Check if event with a particular type and involving a particular character vs character
    pub fn containsCharacter(self: *const LoggingCharacterContactListener, entry_type: Type, character: *const CharacterVirtual, other_character_id: CharacterID) bool {
        return self.findCharacter(entry_type, character, other_character_id) != null;
    }

    /// Find first event with a particular type and involving a particular character vs body and sub shape ID
    pub fn findSubShape(self: *const LoggingCharacterContactListener, entry_type: Type, character: *const CharacterVirtual, body2: BodyID, sub_shape_id2: SubShapeID) ?usize {
        for (self.log.items, 0..) |e, i| {
            if (e.type == entry_type and e.character == character and e.body2.eql(body2) and e.character_id2.isInvalid() and e.sub_shape_id2.eql(sub_shape_id2))
                return i;
        }

        return null;
    }

    /// Check if a particular type and involving a particular character vs body and sub shape ID exists
    pub fn containsSubShape(self: *const LoggingCharacterContactListener, entry_type: Type, character: *const CharacterVirtual, body2: BodyID, sub_shape_id2: SubShapeID) bool {
        return self.findSubShape(entry_type, character, body2, sub_shape_id2) != null;
    }

    /// Find first event with a particular type and involving a particular character vs character and sub shape ID
    pub fn findCharacterSubShape(self: *const LoggingCharacterContactListener, entry_type: Type, character: *const CharacterVirtual, other_character_id: CharacterID, sub_shape_id2: SubShapeID) ?usize {
        for (self.log.items, 0..) |e, i| {
            if (e.type == entry_type and e.character == character and e.body2.isInvalid() and e.character_id2.eql(other_character_id) and e.sub_shape_id2.eql(sub_shape_id2))
                return i;
        }

        return null;
    }

    /// Check if a particular type and involving a particular character vs character and sub shape ID exists
    pub fn containsCharacterSubShape(self: *const LoggingCharacterContactListener, entry_type: Type, character: *const CharacterVirtual, other_character_id: CharacterID, sub_shape_id2: SubShapeID) bool {
        return self.findCharacterSubShape(entry_type, character, other_character_id, sub_shape_id2) != null;
    }
};
