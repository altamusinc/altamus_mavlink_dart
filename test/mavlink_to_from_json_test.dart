import 'package:dart_mavlink/dialects/altamus.dart';
import 'package:test/test.dart';

void main() {
  late Identifier id;
  setUp(() {
      id = Identifier(particleId: [
    48,
    97,
    49,
    48,
    97,
    99,
    101,
    100,
    50,
    48,
    50,
    49,
    57,
    52,
    57,
    52,
    52,
    97,
    48,
    50,
    99,
    48,
    99,
    99
  ], localIp: [
    172,
    16,
    10,
    126
  ], mac: [
    148,
    148,
    74,
    2,
    192,
    204
  ], name: [
    80,
    50,
    45,
    48,
    50,
    67,
    48,
    67,
    67,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  ], siteFriendlyName: [
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  ], siteName: [
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  ]);
  });

  test('Serialize to json then feed back to fromJson constructor', () async {
    var json = id.toJson();
    var id2 = Identifier.fromJson(json);

    expect(id2.particleId, id.particleId);
    expect(id2.localIp, id.localIp);
    expect(id2.mac, id.mac);
    expect(id2.name, id.name);
    expect(id2.siteFriendlyName, id.siteFriendlyName);
    expect(id2.siteName, id.siteName);
  });

  test("Feed mangled data into fromJson constructor and check for exception", () async {
    var mangled = id.toJson();
    mangled["name"] = 50;
    mangled["particleId"] = null;
    expect(() {
      try {
        Identifier.fromJson(mangled);
      } catch (e) {
        throw Exception();
      }
    }, throwsA(isA<Exception>()));
  });
}
