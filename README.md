# cpSerializeJson
Chipmunk2D Json Serialization using nlohmann::json

Simple serialisation and deserialisation using nlohmann::json.

Expects a `json.hpp` somewhere on the path.

## Basic Usage
Serialize an entire space.

```cpp
cpSpace* space;
nlohmann::json json_space = space;
std::cout << json_space.dump() <<std::endl;
```

When deserializing it a new space will be created using `cpSpaceNew`

```cpp
nlohmann::json json_space = R(" json code here ");
cpSpace * space = json_space;
```

## Partial Serialisation/Deserialisation

Getting bodies serialized.

```cpp
cpSpace* space;
cpBody* body =cpBodyNew(space,...);

cpSerializeJsonSetBodySpace(space);
nlohmann::json json_body = body;
std::cout << json_body.dump() <<std::endl;
```

Capturing constraints that are connected to the bodies that have been serialized before.

```cpp
cpSpace* space;
cpBody* bodyA =cpBodyNew(space,...);
cpBody* bodyB =cpBodyNew(space,...);

cpSerializeJsonSetBodySpace(space);
nlohmann::json json_bodies = {bodyA, bodyB};
nlohmann::json json_constraints = cpSerializeJsonBodySerializeConstraints(space);
```