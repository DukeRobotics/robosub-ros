## Essential Functions:
 - **BasicThruGate()** = Order(move("Forward",10)
 - **preQualify()** = Order(submerge(), thruGate(), goAroundMarker(), thruGate())
## Pickup Tasks:
 - **crucifixTask()** = Order(goToCrucifix(), pickupCrucifix())
 - **garlicTask()** = Order(goToGarlic(), pickupGarlic())
## Slay Vampires:
 - **slayVampTask()** = Order(goToSquare(), hitSquare(), goToTriangle(), hitTrangle())
## Drop Tasks:
 - **dropGarlic()** = Order(goToBin(), openBin(), centerOverBin(), drop())
## Stake Through The Heart:
 - **stakeHeart()** = Order(goToVamp(), moveLever(), goToHeart(), shankHeart(), goToOval(), shankOval())
## Expose To Sunlight:
 - **exposeToSun()** = Order(goToCoffins(), openClosedCoffin(), center)verCoffin(), drop(), crucifixTask())
