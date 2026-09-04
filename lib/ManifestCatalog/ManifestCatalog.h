#ifndef MANIFEST_CATALOG_H
#define MANIFEST_CATALOG_H

#include <cstddef>
#include <cstdint>

#include <btp/catalog.hpp>
#include <TelemetryPublisher.h>

// Fills this robot's SERVED btp::Catalog (owned by node_, see BallyRobot.h)
// from TelemetryPublisher::schemas() and a source_info table, so
// btp::Node::serve_catalog() can answer CONTROL/MANIFEST_REQUEST entirely by
// itself (BTP 2.35.0's format-2 source_info + Node::emit_manifest, BTP
// 2.39.0's body-only topics). This is what ManifestResponder used to do by
// hand-encoding MANIFEST_DATA's wire bytes directly (build_manifest_data);
// that class is gone -- btp::Catalog::write_topics() / write_source_info()
// now own the wire layout (BTP/src/catalog.cpp), and populate() below is
// only the one-time translation from this robot's two existing schema
// tables into Catalog::add_topic() / add_source_info() calls.
//
// Deliberately pure C++ (no Arduino/FreeRTOS/btp::Node), the same shape
// ManifestResponder always was, so it links into env:native.
namespace ManifestCatalog {

// One entry of the MANIFEST_DATA source_info block (BTP/docs/commands.md
// section 3.12): a stable machine key, a human label (may be empty), and the
// value as text. All three are borrowed by populate() -- see its own
// comment for the lifetime this requires.
struct SourceInfoEntry {
    const char* key;
    const char* label;
    const char* value;
};

// Upper bound BallyRobot.h keeps in source_info_entries_; commands.md
// section 6 caps the wire at 32, this is this robot's own, smaller budget.
// Must match node_'s StaticNode CatalogSourceInfo template argument.
constexpr std::size_t kMaxSourceInfoEntries = 16U;

// This robot's served catalogue never changes shape at runtime (one static,
// compile-time schema -- same reasoning ManifestResponder::kConfigRevision
// always documented), so config_revision is a compile-time constant, bumped
// whenever a firmware build changes the topics or fields TelemetryPublisher
// exposes. RobotSettings.h's own comment on "the field's documented meaning"
// refers to this constant.
constexpr std::uint32_t kConfigRevision = 2U;

// btp::Role::Producer -- a leaf node describing only itself (see
// bindProtocolTransport()'s serve_catalog() call).
constexpr std::uint8_t kSourceRoleRobot = 0x01U;

// Bound on field-bearing topics/fields this function will load; matches
// node_'s StaticNode CatalogTopics/CatalogFields template arguments
// (BallyRobot.h). A schema exceeding either is a programming error caught at
// boot (see populate()'s return value), the same role ManifestResponder::
// buildCatalog()'s catalog_valid_ used to play.
constexpr std::size_t kMaxCatalogFields = 8U;

// Loads every topic of `schemas`[0..schema_count) into `catalog` via
// Catalog::add_topic() -- a body-only topic (field_count == 0, e.g.
// TelemetryPublisher::kSystemMonitorTopicId's UTF8 sys-health document)
// carries no fields, which BTP 2.39.0's add_topic() now accepts directly, so
// unlike the old buildCatalog() nothing is skipped here: every schema this
// robot publishes ends up in the served catalogue, and therefore on the
// wire. Then loads `source_info`[0..source_info_count) via
// add_source_info() and sets catalog.config_revision() to kConfigRevision.
//
// `schemas` and `source_info` are only read for the duration of this call;
// the STRINGS they point at (topic/field names, units, source_info
// key/label/value) are copied into `catalog`'s own string pool by
// Catalog::add_topic()/add_source_info() (btp::Catalog::intern()), so
// neither array needs to outlive this call the way ManifestResponder's own
// borrowed source_info_ pointer used to.
//
// Returns false the first time a topic or a source_info entry fails to fit
// (a duplicate topic_id, a field whose order skips a position, or `catalog`'s
// pools too small) -- diagnostic only, matching ManifestResponder::
// catalog_ok()'s old contract: the caller logs it, MANIFEST_DATA still
// describes whatever DID fit.
bool populate(btp::Catalog& catalog,
             const TelemetryPublisher::TopicSchema* schemas,
             std::size_t schema_count,
             const SourceInfoEntry* source_info,
             std::size_t source_info_count) noexcept;

}  // namespace ManifestCatalog

#endif  // MANIFEST_CATALOG_H
