# cpJsonSerialize

Chipmunk2D Json Serialization using nlohmann::json
Expects a `json.hpp` somewhere on the path.

nlohmann::json requires at least `C++ 17` and so does this project.

**warning:** I have only tested what I use, so some parts might be missing.

## Basic Usage

Serializing an entire space:

```cpp
cpSpace* space;
nlohmann::json json_space = space;

//print
std::cout << json_space.dump() <<std::endl;

//or save to disk:
auto f = std::ofstream("space.json");
if(f.good())
  f << json_space.dump(2) << std::endl;
```

### deserialization

When deserializing it a new space will be created using `cpSpaceNew()` and all the bodies, shapes and constraints will be created as well. Which means you wil need to remove and free them and destroy the space after using it.

You will also need to set up all your collision handlers after deserializing the space.


```cpp
nlohmann::json json_space = R("<json>");

cpSpace * space = nullptr;

// make sure to destroy a space that has been created before
if(space)
  cpDestroy(space);

space = json_space;
```

Keep in mind though that `cpDataPointer userData`, that has been put into `cpSpace`, `cpBody`, `cpShape` or `cpConstraint`, will be stored as an unsigned integer and restored as such, if you used `userData` to store pointers, you will need to reconstruct them somehow.
If you are using a custom type with `CP_DATA_POINTER_TYPE` let me know.

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

## Serializing pointers

Pointers to bodies and constraints will be serialized to `{"ptr": "<hexstring>"}` and recovered in deserialization. If for some reason you want to serialize only the pointers you can set:

```cpp
cpJSSetSerializeAsPtr(true);
// serialization will only serialize pointers
cpJSSetSerializeAsPtr(false);
// serialization will serialize all properties
```