```git submodule update --init --recursive``` to fetch required git submodule holding our forked message definitions.

```flutter pub get``` to install depenencies

```dart run ./tool/generate.dart -d mavlink/message_definitions/v1.0/altamus.xml``` to generate files

