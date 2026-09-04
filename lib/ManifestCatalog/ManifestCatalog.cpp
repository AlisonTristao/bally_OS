#include <ManifestCatalog.h>

#include <btp/messages.hpp>

#include <cstring>

namespace {

btp::ByteView view_of(const char* text) noexcept {
    return {reinterpret_cast<const std::uint8_t*>(text),
            (text == nullptr) ? 0U : std::strlen(text)};
}

}  // namespace

namespace ManifestCatalog {

bool populate(btp::Catalog& catalog,
             const TelemetryPublisher::TopicSchema* schemas,
             std::size_t schema_count,
             const SourceInfoEntry* source_info,
             std::size_t source_info_count) noexcept {
    catalog.set_config_revision(kConfigRevision);

    bool ok = true;

    for (std::size_t t = 0U; t < schema_count; ++t) {
        const TelemetryPublisher::TopicSchema& topic = schemas[t];

        if (topic.field_count == 0U) {
            // Body-only topic (kSystemMonitorTopicId's UTF8 document): no
            // FieldRecord to build, BTP 2.39.0 accepts fields=nullptr here.
            const btp::MessageError err = catalog.add_topic(
                topic.topic_id, topic.schema_version,
                static_cast<btp::TelemetryEncoding>(topic.encoding),
                /*subscribable=*/true, topic.max_rate_millihz, topic.name,
                nullptr, 0U);
            if (err != btp::MessageError::Ok) ok = false;
            continue;
        }

        if (topic.field_count > kMaxCatalogFields) {
            ok = false;
            continue;
        }
        btp::FieldRecord records[kMaxCatalogFields];
        for (std::size_t f = 0U; f < topic.field_count; ++f) {
            const TelemetryPublisher::FieldSchema& field = topic.fields[f];
            btp::FieldRecord& fr = records[f];
            fr = {};
            fr.field_id = field.field_id;
            fr.order = field.order;
            fr.type = static_cast<std::uint8_t>(field.type);
            fr.flags = field.nullable ? btp::kFieldNullable
                                      : static_cast<std::uint8_t>(0U);
            fr.element_count = field.element_count;
            fr.max_element_count = 0U;
            fr.scale = static_cast<double>(field.scale);
            fr.offset = static_cast<double>(field.offset);
            fr.enum_count = 0U;
            fr.name = view_of(field.name);
            fr.unit = view_of(field.unit);
        }
        const btp::MessageError err = catalog.add_topic(
            topic.topic_id, topic.schema_version,
            static_cast<btp::TelemetryEncoding>(topic.encoding),
            /*subscribable=*/true, topic.max_rate_millihz, topic.name,
            records, topic.field_count);
        if (err != btp::MessageError::Ok) ok = false;
    }

    for (std::size_t i = 0U; i < source_info_count; ++i) {
        const SourceInfoEntry& entry = source_info[i];
        if (entry.value == nullptr || entry.value[0] == '\0') continue;
        if (catalog.add_source_info(entry.key, entry.label, entry.value) !=
            btp::MessageError::Ok) {
            ok = false;
        }
    }

    return ok;
}

}  // namespace ManifestCatalog
