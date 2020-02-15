#Only works with Python3
class LearnPython:
    def printer(self):
        return self.num

    def __init__(self,num):
        self.num = num



instanceObj = LearnPython(12345)
print(instanceObj.printer())

class ExtenderClass(LearnPython):
    def __init__(self, num):
        super().__init__(num) #super() returns an object of the parent class


extendedInst = ExtenderClass(1111)
print(extendedInst.printer())
