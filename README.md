# cpJsonSerialize
Chipmunk2D Json Serialization using nlohmann::json
Expects a `json.hpp` somewhere on the path.

nlohmann::json requires at least `C++ 17` and so does this project.

## Basic Usage
Serializing an entire space:

```cpp
cpSpace* space;
nlohmann::json json_space = space;
std::cout << json_space.dump() <<std::endl;
```

When deserializing it a new space will be created using `cpSpaceNew()` and all the bodies, shapes and constraints will be created as well. Which means you wil need to remove and free them and destroy the space after using it.

You will also need to set up all your collision handlers after deserializing the space.


```cpp
nlohmann::json json_space = R(" json code here ");
cpSpace * space = json_space;
```

Keep in mind though that userdata, that has been put into the space, the bodies, shapes or constraints, will be stored as an unsigned integer and restored as such, if you used userdata to store pointers, you will need to reconstruct them somehow.


## Partial Serialisation/Deserialisation

You can also serialize snd deserialize single bodies, shapes and constraints and most of the chipmunk structs such as `cpVect` or `cpBB`

Getting bodies serialized.

```cpp
cpSpace* space;
cpBody* body = cpBodyNew(space,...);

cpJSSetBodySpace(space);
nlohmann::json json_body = body;
std::cout << json_body.dump() <<std::endl;
```

Capturing constraints that are connected to the bodies that have been serialized before.

```cpp
cpSpace* space;
cpBody* bodyA =cpBodyNew(space,...);
cpBody* bodyB =cpBodyNew(space,...);

cpJSSetBodySpace(space);
nlohmann::json json_bodies = {bodyA, bodyB};
nlohmann::json json_constraints = cpJSBodySerializeConstraints(space);
```