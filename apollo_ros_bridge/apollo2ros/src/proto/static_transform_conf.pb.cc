// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: static_transform_conf.proto

#include "static_transform_conf.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
extern PROTOBUF_INTERNAL_EXPORT_static_5ftransform_5fconf_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ExtrinsicFile_static_5ftransform_5fconf_2eproto;
namespace apollo {
namespace static_transform {
class ExtrinsicFileDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ExtrinsicFile> _instance;
} _ExtrinsicFile_default_instance_;
class ConfDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Conf> _instance;
} _Conf_default_instance_;
}  // namespace static_transform
}  // namespace apollo
static void InitDefaultsscc_info_Conf_static_5ftransform_5fconf_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::static_transform::_Conf_default_instance_;
    new (ptr) ::apollo::static_transform::Conf();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::static_transform::Conf::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Conf_static_5ftransform_5fconf_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_Conf_static_5ftransform_5fconf_2eproto}, {
      &scc_info_ExtrinsicFile_static_5ftransform_5fconf_2eproto.base,}};

static void InitDefaultsscc_info_ExtrinsicFile_static_5ftransform_5fconf_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::static_transform::_ExtrinsicFile_default_instance_;
    new (ptr) ::apollo::static_transform::ExtrinsicFile();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::static_transform::ExtrinsicFile::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ExtrinsicFile_static_5ftransform_5fconf_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_ExtrinsicFile_static_5ftransform_5fconf_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_static_5ftransform_5fconf_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_static_5ftransform_5fconf_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_static_5ftransform_5fconf_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_static_5ftransform_5fconf_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::static_transform::ExtrinsicFile, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::static_transform::ExtrinsicFile, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::static_transform::ExtrinsicFile, frame_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::static_transform::ExtrinsicFile, child_frame_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::static_transform::ExtrinsicFile, file_path_),
  PROTOBUF_FIELD_OFFSET(::apollo::static_transform::ExtrinsicFile, enable_),
  0,
  1,
  2,
  3,
  PROTOBUF_FIELD_OFFSET(::apollo::static_transform::Conf, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::static_transform::Conf, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::static_transform::Conf, extrinsic_file_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::apollo::static_transform::ExtrinsicFile)},
  { 13, 19, sizeof(::apollo::static_transform::Conf)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::static_transform::_ExtrinsicFile_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::static_transform::_Conf_default_instance_),
};

const char descriptor_table_protodef_static_5ftransform_5fconf_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\033static_transform_conf.proto\022\027apollo.st"
  "atic_transform\"\\\n\rExtrinsicFile\022\020\n\010frame"
  "_id\030\001 \001(\t\022\026\n\016child_frame_id\030\002 \001(\t\022\021\n\tfil"
  "e_path\030\003 \001(\t\022\016\n\006enable\030\004 \001(\010\"F\n\004Conf\022>\n\016"
  "extrinsic_file\030\001 \003(\0132&.apollo.static_tra"
  "nsform.ExtrinsicFile"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_static_5ftransform_5fconf_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_static_5ftransform_5fconf_2eproto_sccs[2] = {
  &scc_info_Conf_static_5ftransform_5fconf_2eproto.base,
  &scc_info_ExtrinsicFile_static_5ftransform_5fconf_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_static_5ftransform_5fconf_2eproto_once;
static bool descriptor_table_static_5ftransform_5fconf_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_static_5ftransform_5fconf_2eproto = {
  &descriptor_table_static_5ftransform_5fconf_2eproto_initialized, descriptor_table_protodef_static_5ftransform_5fconf_2eproto, "static_transform_conf.proto", 220,
  &descriptor_table_static_5ftransform_5fconf_2eproto_once, descriptor_table_static_5ftransform_5fconf_2eproto_sccs, descriptor_table_static_5ftransform_5fconf_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_static_5ftransform_5fconf_2eproto::offsets,
  file_level_metadata_static_5ftransform_5fconf_2eproto, 2, file_level_enum_descriptors_static_5ftransform_5fconf_2eproto, file_level_service_descriptors_static_5ftransform_5fconf_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_static_5ftransform_5fconf_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_static_5ftransform_5fconf_2eproto), true);
namespace apollo {
namespace static_transform {

// ===================================================================

void ExtrinsicFile::InitAsDefaultInstance() {
}
class ExtrinsicFile::_Internal {
 public:
  using HasBits = decltype(std::declval<ExtrinsicFile>()._has_bits_);
  static void set_has_frame_id(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_child_frame_id(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_file_path(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_enable(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

ExtrinsicFile::ExtrinsicFile()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.static_transform.ExtrinsicFile)
}
ExtrinsicFile::ExtrinsicFile(const ExtrinsicFile& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  frame_id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_frame_id()) {
    frame_id_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.frame_id_);
  }
  child_frame_id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_child_frame_id()) {
    child_frame_id_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.child_frame_id_);
  }
  file_path_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_file_path()) {
    file_path_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.file_path_);
  }
  enable_ = from.enable_;
  // @@protoc_insertion_point(copy_constructor:apollo.static_transform.ExtrinsicFile)
}

void ExtrinsicFile::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_ExtrinsicFile_static_5ftransform_5fconf_2eproto.base);
  frame_id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  child_frame_id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  file_path_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  enable_ = false;
}

ExtrinsicFile::~ExtrinsicFile() {
  // @@protoc_insertion_point(destructor:apollo.static_transform.ExtrinsicFile)
  SharedDtor();
}

void ExtrinsicFile::SharedDtor() {
  frame_id_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  child_frame_id_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  file_path_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void ExtrinsicFile::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ExtrinsicFile& ExtrinsicFile::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ExtrinsicFile_static_5ftransform_5fconf_2eproto.base);
  return *internal_default_instance();
}


void ExtrinsicFile::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.static_transform.ExtrinsicFile)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      frame_id_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000002u) {
      child_frame_id_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000004u) {
      file_path_.ClearNonDefaultToEmptyNoArena();
    }
  }
  enable_ = false;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* ExtrinsicFile::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional string frame_id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParserUTF8Verify(_internal_mutable_frame_id(), ptr, ctx, "apollo.static_transform.ExtrinsicFile.frame_id");
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional string child_frame_id = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParserUTF8Verify(_internal_mutable_child_frame_id(), ptr, ctx, "apollo.static_transform.ExtrinsicFile.child_frame_id");
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional string file_path = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParserUTF8Verify(_internal_mutable_file_path(), ptr, ctx, "apollo.static_transform.ExtrinsicFile.file_path");
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool enable = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          _Internal::set_has_enable(&has_bits);
          enable_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* ExtrinsicFile::InternalSerializeWithCachedSizesToArray(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.static_transform.ExtrinsicFile)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string frame_id = 1;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_frame_id().data(), static_cast<int>(this->_internal_frame_id().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.static_transform.ExtrinsicFile.frame_id");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_frame_id(), target);
  }

  // optional string child_frame_id = 2;
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_child_frame_id().data(), static_cast<int>(this->_internal_child_frame_id().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.static_transform.ExtrinsicFile.child_frame_id");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_child_frame_id(), target);
  }

  // optional string file_path = 3;
  if (cached_has_bits & 0x00000004u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_file_path().data(), static_cast<int>(this->_internal_file_path().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.static_transform.ExtrinsicFile.file_path");
    target = stream->WriteStringMaybeAliased(
        3, this->_internal_file_path(), target);
  }

  // optional bool enable = 4;
  if (cached_has_bits & 0x00000008u) {
    stream->EnsureSpace(&target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(4, this->_internal_enable(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.static_transform.ExtrinsicFile)
  return target;
}

size_t ExtrinsicFile::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.static_transform.ExtrinsicFile)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional string frame_id = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_frame_id());
    }

    // optional string child_frame_id = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_child_frame_id());
    }

    // optional string file_path = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_file_path());
    }

    // optional bool enable = 4;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 1;
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ExtrinsicFile::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.static_transform.ExtrinsicFile)
  GOOGLE_DCHECK_NE(&from, this);
  const ExtrinsicFile* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ExtrinsicFile>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.static_transform.ExtrinsicFile)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.static_transform.ExtrinsicFile)
    MergeFrom(*source);
  }
}

void ExtrinsicFile::MergeFrom(const ExtrinsicFile& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.static_transform.ExtrinsicFile)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _has_bits_[0] |= 0x00000001u;
      frame_id_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.frame_id_);
    }
    if (cached_has_bits & 0x00000002u) {
      _has_bits_[0] |= 0x00000002u;
      child_frame_id_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.child_frame_id_);
    }
    if (cached_has_bits & 0x00000004u) {
      _has_bits_[0] |= 0x00000004u;
      file_path_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.file_path_);
    }
    if (cached_has_bits & 0x00000008u) {
      enable_ = from.enable_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ExtrinsicFile::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.static_transform.ExtrinsicFile)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ExtrinsicFile::CopyFrom(const ExtrinsicFile& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.static_transform.ExtrinsicFile)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ExtrinsicFile::IsInitialized() const {
  return true;
}

void ExtrinsicFile::InternalSwap(ExtrinsicFile* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  frame_id_.Swap(&other->frame_id_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  child_frame_id_.Swap(&other->child_frame_id_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  file_path_.Swap(&other->file_path_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(enable_, other->enable_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ExtrinsicFile::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void Conf::InitAsDefaultInstance() {
}
class Conf::_Internal {
 public:
  using HasBits = decltype(std::declval<Conf>()._has_bits_);
};

Conf::Conf()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.static_transform.Conf)
}
Conf::Conf(const Conf& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      extrinsic_file_(from.extrinsic_file_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:apollo.static_transform.Conf)
}

void Conf::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Conf_static_5ftransform_5fconf_2eproto.base);
}

Conf::~Conf() {
  // @@protoc_insertion_point(destructor:apollo.static_transform.Conf)
  SharedDtor();
}

void Conf::SharedDtor() {
}

void Conf::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Conf& Conf::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Conf_static_5ftransform_5fconf_2eproto.base);
  return *internal_default_instance();
}


void Conf::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.static_transform.Conf)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  extrinsic_file_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

const char* Conf::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .apollo.static_transform.ExtrinsicFile extrinsic_file = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_extrinsic_file(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10>(ptr));
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Conf::InternalSerializeWithCachedSizesToArray(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.static_transform.Conf)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.static_transform.ExtrinsicFile extrinsic_file = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_extrinsic_file_size()); i < n; i++) {
    stream->EnsureSpace(&target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessageToArray(1, this->_internal_extrinsic_file(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.static_transform.Conf)
  return target;
}

size_t Conf::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.static_transform.Conf)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.static_transform.ExtrinsicFile extrinsic_file = 1;
  total_size += 1UL * this->_internal_extrinsic_file_size();
  for (const auto& msg : this->extrinsic_file_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Conf::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.static_transform.Conf)
  GOOGLE_DCHECK_NE(&from, this);
  const Conf* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Conf>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.static_transform.Conf)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.static_transform.Conf)
    MergeFrom(*source);
  }
}

void Conf::MergeFrom(const Conf& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.static_transform.Conf)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  extrinsic_file_.MergeFrom(from.extrinsic_file_);
}

void Conf::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.static_transform.Conf)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Conf::CopyFrom(const Conf& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.static_transform.Conf)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Conf::IsInitialized() const {
  return true;
}

void Conf::InternalSwap(Conf* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  extrinsic_file_.InternalSwap(&other->extrinsic_file_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Conf::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace static_transform
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::static_transform::ExtrinsicFile* Arena::CreateMaybeMessage< ::apollo::static_transform::ExtrinsicFile >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::static_transform::ExtrinsicFile >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::static_transform::Conf* Arena::CreateMaybeMessage< ::apollo::static_transform::Conf >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::static_transform::Conf >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
