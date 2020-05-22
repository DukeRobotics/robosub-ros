## Essential Functions:
 - BasicThruGate() = Order(move("Forward",10)
 - preQualify() = Order(submerge(), thruGate(), goAroundMarker(), thruGate())
## Pickup Tasks:
 - crucifixTask() = Order(findCrucifix(), goTocrucifix(), pickupCrucifix())
 - garlicTask() = Order(findGarlic(), goToGarlic(), pickupGarlic())
## Slay Vampires:
 - slayVampTask() = Order(findSquare(), hitSquare(), findTriangle(), hitTrangle())
## Drop Tasks:
 - dropGarlic = Order(findBin() 
