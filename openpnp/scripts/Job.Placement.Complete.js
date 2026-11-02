/*
 * Release the nozzle rotation drivers once both nozzles are free. The
 * two rotations share one power supply, and the shafts are first turned
 * back to 0 degrees, which is on a full-step detent, so they keep their
 * angle while unpowered.
 */

function holdRot() {
    var a = machine.getDefaultHead().getActuatorByName("HOLD_ROT");
    if (a === null) {
        a = machine.getActuatorByName("HOLD_ROT");
    }
    if (a === null) {
        throw new java.lang.Exception("HOLD_ROT actuator not found, see SETUP.md");
    }
    return a;
}

var head = machine.getDefaultHead();
var nozzles = head.getNozzles();

var busy = false;
for (var i = 0; i < nozzles.size(); i++) {
    if (nozzles.get(i).getPart() != null) {
        busy = true;
    }
}

if (!busy) {
    for (var i = 0; i < nozzles.size(); i++) {
        var n = nozzles.get(i);
        n.moveToSafeZ();
        n.moveTo(n.getLocation().derive(null, null, null, 0.0));
    }

    holdRot().actuate(false);
}
