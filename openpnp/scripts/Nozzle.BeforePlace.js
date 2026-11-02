/*
 * Power the nozzle rotation drivers before the place. OpenPnP closes the
 * valve while the nozzle is still down on the part, and an unpowered
 * shaft would turn with the part on the way up. M800 E1 keeps the power
 * on until Job.Placement.Complete.js releases it.
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

holdRot().actuate(true);
