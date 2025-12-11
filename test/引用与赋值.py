class SHIT:
    def __init__(self):
        self.status={"x":0,"y":0,"z":1}

    def shit(self):
        print(self.status)
        status = self.status.copy()
        status["x"] = 1
        print(self.status)

a=SHIT()
a.shit()

class SHIT2:
    def __init__(self):
        self.status=[1,2,3]

    def shit2(self):
        print(self.status)
        status = self.status
        status[0] = 0
        print(self.status)

a=SHIT2()
a.shit2()


class SHIT3:
    def __init__(self):
        self.status=1

    def shit3(self):
        print(self.status)
        status = self.status
        status = 0
        print(self.status)

a=SHIT3()
a.shit3()


class SHIT4:
    def __init__(self):
        self.status="asdasdas"

    def shit4(self):
        print(self.status)
        status = self.status
        status = "1"
        print(self.status)

a=SHIT4()
a.shit4()

class SHIT6():
    def shit6(self):
        yield "你好"
        yield "今天过得怎么样"
        yield "辛苦啦"
a=SHIT6()
result=a.shit6()
print(next(result))
print(next(result))
print(next(result))