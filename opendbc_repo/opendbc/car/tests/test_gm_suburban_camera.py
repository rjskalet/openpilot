from opendbc.car.car_helpers import _normalize_gm_suburban_camera_candidate
from opendbc.car.gm.values import ALT_ACCS, CAMERA_ACC_CAR, CAR


SUBURBAN_VIN = "1GNSKJKJ9KR123456"
YUKON_VIN = "1GKS2CKJ9KR123456"

MATCHING_FINGERPRINT = {
  0: {
    190: 6,
    201: 8,
    209: 7,
    211: 2,
    241: 6,
    304: 1,
    320: 3,
  },
}


def test_suburban_camera_from_yukon_candidate():
  assert _normalize_gm_suburban_camera_candidate("GMC_YUKON", MATCHING_FINGERPRINT, SUBURBAN_VIN) == \
         "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"


def test_suburban_camera_from_ambiguous_candidate():
  assert _normalize_gm_suburban_camera_candidate(None, MATCHING_FINGERPRINT, SUBURBAN_VIN) == \
         "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"


def test_suburban_camera_does_not_require_transient_camera_diagnostics():
  # Camera diagnostic request/response IDs are not guaranteed to appear in the short
  # passive fingerprint window. VIN + the stable PT signature must still resolve it.
  fingerprint = {0: MATCHING_FINGERPRINT[0].copy(), 2: {}}
  assert _normalize_gm_suburban_camera_candidate("GMC_YUKON", fingerprint, SUBURBAN_VIN) == \
         "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"


def test_yukon_vin_stays_yukon():
  assert _normalize_gm_suburban_camera_candidate("GMC_YUKON", MATCHING_FINGERPRINT, YUKON_VIN) == "GMC_YUKON"


def test_wrong_powertrain_signature_does_not_match():
  fingerprint = {bus: messages.copy() for bus, messages in MATCHING_FINGERPRINT.items()}
  fingerprint[0][304] = 8
  assert _normalize_gm_suburban_camera_candidate("GMC_YUKON", fingerprint, SUBURBAN_VIN) == "GMC_YUKON"


def test_suburban_camera_uses_camera_acc_alt_acc_path():
  platform = CAR.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN
  assert platform in CAMERA_ACC_CAR
  assert platform in ALT_ACCS

def test_mazda_candidate_is_not_changed_by_suburban_disambiguation():
  from opendbc.car.mazda.values import CAR as MAZDA
  assert _normalize_gm_suburban_camera_candidate(MAZDA.MAZDA_CX9, MATCHING_FINGERPRINT, SUBURBAN_VIN) == MAZDA.MAZDA_CX9


def test_suburban_signature_requires_every_powertrain_frame():
  for address in MATCHING_FINGERPRINT[0]:
    missing = {0: {k: v for k, v in MATCHING_FINGERPRINT[0].items() if k != address}}
    assert _normalize_gm_suburban_camera_candidate("GMC_YUKON", missing, SUBURBAN_VIN) == "GMC_YUKON"


def test_suburban_signature_must_be_on_powertrain_bus_zero():
  fingerprints = {1: MATCHING_FINGERPRINT[0].copy(), 0: {}}
  assert _normalize_gm_suburban_camera_candidate("GMC_YUKON", fingerprints, SUBURBAN_VIN) == "GMC_YUKON"


def test_get_car_normalizes_suburban_but_not_fixed_selection_or_mazda(monkeypatch):
  from types import SimpleNamespace
  import opendbc.car.car_helpers as helpers
  from opendbc.car.structs import CarParams
  from opendbc.car.mazda.values import CAR as MAZDA

  class FakeInterface:
    def __init__(self, cp, cp_sp):
      self.CP = cp
      self.CP_SP = cp_sp

    @staticmethod
    def get_params(candidate, *args, **kwargs):
      return SimpleNamespace(carFingerprint=candidate)

    @staticmethod
    def get_params_sp(*args, **kwargs):
      return SimpleNamespace()

  monkeypatch.setattr(helpers, "interfaces", {
    "GMC_YUKON": FakeInterface,
    "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN": FakeInterface,
    MAZDA.MAZDA_CX9: FakeInterface,
  })
  monkeypatch.setattr(helpers, "sunnypilot_interfaces", lambda *args, **kwargs: None)

  for candidate, source, vin, expected in [
    ("GMC_YUKON", CarParams.FingerprintSource.can, SUBURBAN_VIN, "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"),
    (None, CarParams.FingerprintSource.can, SUBURBAN_VIN, "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"),
    ("GMC_YUKON", CarParams.FingerprintSource.fixed, SUBURBAN_VIN, "GMC_YUKON"),
    ("GMC_YUKON", CarParams.FingerprintSource.can, YUKON_VIN, "GMC_YUKON"),
    (MAZDA.MAZDA_CX9, CarParams.FingerprintSource.can, SUBURBAN_VIN, MAZDA.MAZDA_CX9),
  ]:
    monkeypatch.setattr(
      helpers, "fingerprint",
      lambda *args, _candidate=candidate, _source=source, _vin=vin, **kwargs:
        (_candidate, MATCHING_FINGERPRINT, _vin, [], _source, True),
    )
    ci = helpers.get_car(None, None, None, False, False)
    assert ci.CP.carFingerprint == expected
    assert ci.CP.fingerprintSource == source
