class Test():
    def __init__(self, aa="123", bb=123):
        self.save1 = aa
        self.save2 = bb

    def testPrint(self):
        print(self.save1,self.save2)

a = Test()
b = Test("qweqwe",123123)
c = Test("rkskek")

a.testPrint()
b.testPrint()
c.testPrint()

