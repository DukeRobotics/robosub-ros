## Essential Functions:
 - **basicThruGate()** = Order(moveForward(10))
 - **preQualify()** = Order(submerge(), thruGate(), goAroundMarker(), thruGate())
	- submerge() = Order(moveDown())
	- thruGate() = Order(gotoGate(), moveThru())
		- gotoGate() = Order(findGate(), move(toPos=gatePoint))
			- findGate() a computer vision function that returns gatePoint
	- goAroundMarker() = Order(findMarker(), move(toPos=markerPoint), circleAround(), findGate()
## Pickup Tasks:
 - **crucifixTask()** = Order(gotoCrucifix(), pickupCrucifix())
 - **garlicTask()** = Order(gotoGarlic(), pickupGarlic())
## Slay Vampires:
 - **slayVampTask()** = Order(gotoSquare(), hitSquare(), gotoTriangle(), hitTrangle())
## Drop Tasks:
 - **dropGarlic()** = Order(gotoBin(), openBin(), centerOverBin(), drop())
## Stake Through The Heart:
 - **stakeHeart()** = Order(gotoVamp(), moveLever(), gotoHeart(), shankHeart(), gotoOval(), shankOval())
## Expose To Sunlight:
 - **exposeToSun()** = Order(gotoCoffins(), openClosedCoffin(), center)verCoffin(), drop(), crucifixTask())
