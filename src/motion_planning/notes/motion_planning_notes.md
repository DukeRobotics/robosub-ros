## Essential Functions:
 - **basicThruGate()** = Order(moveForward(10))
 - **preQualify()** = Order(submerge(), thruGate(), goAroundMarker(), thruGate())
	- submerge() = Order(moveDown())
	- thruGate() = Order(gotoGate(), moveThru())
		- gotoGate() = Order(findGate(), move(toPos=gatePoint))
			- findGate() return gatePoint
	- goAroundMarker() = Order(findMarker(), move(toPos=markerPoint), circleAround(), findGate()
		- findMarker() returns markerPoint
		- circleAround() = Order(makeWaypoints(), gotoWaypoints())
			- makeWaypoints() returns list of waypoints that forms a square around marker
			- gotoWaypoints() = move(waypoints[i]) for each waypoint
## Pickup Tasks:
 - **crucifixTask()** = Order(gotoCrucifix(), pickupCrucifix())
	- gotoCrucifix() = Order(findCrucifix(), move(crucifixPoint))
	- pickupCrucifix() = Order(faceCrucifix(), pickup())
 - **garlicTask()** = Order(gotoGarlic(), pickupGarlic())
	- gotoGarlic() = Order(findGarlic(), move(garlicPoint))
	- pickupGarlic() = Order(faceGarlic(), pickup())
## Slay Vampires:
 - **slayVampTask()** = Order(gotoSquare(), hitSquare(), gotoTriangle(), hitTriangle())
	- gotoSquare() = Order(findSquare(), move(squarePoint))
	- hitSquare() = Order(move(squarePoint + delta), move(squarePoint - delta))
	- gotoTriangle() = Order(findTriangle(), move(trianglePoint))
	- hitTriangle() = Order(move(trianglePoint + delta), move(trianglePoint - delta)) 
## Drop Tasks:
 - **dropGarlic()** = Order(gotoBin(), openBin(), centerOverBin(), drop())
## Stake Through The Heart:
 - **stakeHeart()** = Order(gotoVamp(), moveLever(), gotoHeart(), shankHeart(), gotoOval(), shankOval())
## Expose To Sunlight:
 - **exposeToSun()** = Order(gotoCoffins(), openClosedCoffin(), center)verCoffin(), drop(), crucifixTask())
