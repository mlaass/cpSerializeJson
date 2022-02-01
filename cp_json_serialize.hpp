#ifndef CP_SERIALIZE_JSON_HPP
#define CP_SERIALIZE_JSON_HPP

#include "json.hpp"
#include <chipmunk/chipmunk.h>
#include <chipmunk/chipmunk_structs.h>
#include <chipmunk/chipmunk_types.h>

extern void cpJSSetShapeBody(cpBody *body);
extern void cpJSBodySetSerializeAsPtr(bool asPtr);
extern void cpJSConstraintSetSerializeAsPtr(bool asPtr);
extern void cpJSSetSerializeAsPtr(bool asPtr);
extern void cpJSSetBodySpace(cpSpace *space);
extern void cpJSSetShapeSpace(cpSpace *space);
extern void cpJSClearBodyConstraintMap();
extern nlohmann::json cpJSBodySerializeConstraints(cpSpace *space);
extern void cpJSBodyDeserializeConstraints(cpSpace *space,
                                           const nlohmann::json &contraints);

namespace nlohmann {

template <> struct adl_serializer<struct cpTransform> {
  static void to_json(json &j, const struct cpTransform &t);
  static void from_json(const json &j, cpTransform &t);
};

template <> struct adl_serializer<struct cpVect> {
  static void to_json(json &j, const struct cpVect &v);
  static void from_json(const json &j, cpVect &v);
};

template <> struct adl_serializer<struct cpShapeFilter> {
  static void to_json(json &j, const struct cpShapeFilter &f);
  static void from_json(const json &j, cpShapeFilter &f);
};

template <> struct adl_serializer<struct cpBB> {
  static void to_json(json &j, const struct cpBB &bb);
  static void from_json(const json &j, cpBB &bb);
};

template <> struct adl_serializer<struct cpShapeMassInfo> {
  static void to_json(json &j, const struct cpShapeMassInfo &mi);
  static void from_json(const json &j, cpShapeMassInfo &mi);
};

template <> struct adl_serializer<struct cpShape *> {
  static cpBody *body;
  static cpSpace *space;

  static void to_json(json &j, const struct cpShape *shape);
  static cpShape *from_json(const json &j);
};

template <> struct adl_serializer<struct cpBody *> {
  static cpSpace *space;
  static std::map<std::string, cpConstraint *> constraintMap;
  static std::map<std::string, cpBody *> bodyMap;
  static bool asPtrStr;

  static void to_json(json &j, const struct cpBody *body);
  static cpBody *from_json(const json &j);
};

template <> struct adl_serializer<struct cpConstraint *> {
  static cpSpace *space;
  static std::map<std::string, cpConstraint *> constraintMap;
  static std::map<std::string, cpBody *> *bodyMap;
  static bool asPtrStr;

  static void getConstraintType(json &j, const cpConstraint *ct);
  static void to_json(json &j, const cpConstraint *ct);
  static cpConstraint *from_json(const json &j);
};

template <> struct adl_serializer<struct cpSpace *> {
  static void to_json(json &j, const struct cpSpace *space);
  static cpSpace *from_json(const json &j);
};

} // namespace nlohmann

#endif