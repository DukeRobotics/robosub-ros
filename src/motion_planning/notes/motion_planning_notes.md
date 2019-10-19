## Essential Functions:
 - **basicThruGate()** = Order(moveForward(10))
 - **preQualify()** = Order(submerge(), thruGate(), goAroundMarker(), thruGate())
	- submerge() = Order(moveDown())
	- thruGate() = Order(goToGate(), moveThru())
		- gotoGate() = Order(findGate(), findDistToGate(), )
	- goAroundMarker() = Order(findMarker(), findDistToMarker(), moveToMarker(), circleAround(), findGate()
## Pickup Tasks:
 - **crucifixTask()** = Order(gotoCrucifix(), pickupCrucifix())
 - **garlicTask()** = Order(goToGarlic(), pickupGarlic())
## Slay Vampires:
 - **slayVampTask()** = Order(goToSquare(), hitSquare(), goToTriangle(), hitTrangle())
## Drop Tasks:
 - **dropGarlic()** = Order(goToBin(), openBin(), centerOverBin(), drop())
## Stake Through The Heart:
 - **stakeHeart()** = Order(goToVamp(), moveLever(), goToHeart(), shankHeart(), goToOval(), shankOval())
## Expose To Sunlight:
 - **exposeToSun()** = Order(goToCoffins(), openClosedCoffin(), center)verCoffin(), drop(), crucifixTask())
