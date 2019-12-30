#!/usr/bin/python3

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner
from autobahn import wamp

import math

class Component(ApplicationSession):
    def __init__(self, config=None):
        ApplicationSession.__init__(self, config)
        self.gyro = []
        self.accel = []
        self.mag = []

        self.accel_range = { "x":[0, 0], "y":[0, 0], "z":[0, 0] }
        self.accel_sigma = 100
        self.gyro_sigma = 100

    @wamp.subscribe(u'mag.raw')
    def on_mag(self, i):
        self.mag.insert(0, i)
        self.mag = self.mag[:32]

    def accel_need(self):
        need = ""

        for axis in ["x", "y", "z"]:
            for n in range(2):
                if self.accel_range[axis][n] == 0:
                    if n == 0:
                        need += "+"
                    else:
                        need += "-"
                    need += axis + " "

        return need

    @wamp.subscribe(u'accel.raw')
    def on_accel(self, i):
        self.accel.insert(0, i)

        if len(self.accel) > 31:
            self.accel = self.accel[:32]
            sigma, avr = self.sigma(self.accel)

            if sigma > self.accel_sigma * 300:
                self.accel_sigma = 100
                print("Accel reset")
            elif sigma < self.accel_sigma:
                self.accel_sigma = sigma

                r = math.sqrt(math.pow(avr[0], 2) + math.pow(avr[1], 2) + math.pow(avr[2], 2))
                e = math.cos(5.0 * 3.141526 / 180.0);
                norm = [0, 0, 0]
                need = self.accel_need()

                for i in range(3):
                   norm[i] = avr[i] / r

                if norm[0] > e:
                    self.accel_range["x"][0] = r;
                    print("Accel +x {} {}".format(sigma, need))
                elif norm[1] > e:
                    self.accel_range["y"][0] = r;
                    print("Accel +y {} {}".format(sigma, need))
                elif norm[2] > e:
                    self.accel_range["z"][0] = r;
                    print("Accel +z {} {}".format(sigma, need))
                elif norm[0] < -e:
                    self.accel_range["x"][1] = r;
                    print("Accel -x {} {}".format(sigma, need))
                elif norm[1] < -e:
                    self.accel_range["y"][1] = r;
                    print("Accel -y {} {}".format(sigma, need))
                elif norm[2] < -e:
                    self.accel_range["z"][1] = r;
                    print("Accel -z {} {}".format(sigma, need))

    @inlineCallbacks
    @wamp.subscribe(u'gyro.raw')
    def on_gyro(self, i):
        self.gyro.insert(0, i)

        if len(self.gyro) > 31:
            self.gyro = self.gyro[:32]
            sigma, avr = self.sigma(self.gyro)

            if sigma < self.gyro_sigma:
                self.gyro_sigma = sigma
                print("Gyro {}".format(avr))
                yield self.call(u'LoadConfig', gyro=avr)

    def avr(self, l):
        x = 0
        y = 0
        z = 0

        for i in l:
            x += i["x"]
            y += i['y']
            z += i["z"]

        return [ x/len(l), y/len(l), z/len(l) ]

    def sigma(self, l):
        a = self.avr(l)
        s = [0, 0, 0]
        r = 0

        for i in l:
            s[0] += math.pow(a[0] - i["x"], 2)
            s[1] += math.pow(a[1] - i["y"], 2)
            s[2] += math.pow(a[2] - i["z"], 2)

        for i in range(3):
            r += s[i] / len(l)

        return math.sqrt(r), a

    @inlineCallbacks
    def onJoin(self, details):
        print("session attached")

        res = yield self.call(u'SetFreq', freq=100)
        print("Answer: {}".format(res))

        results = yield self.subscribe(self)

    def onDisconnect(self):
        print("disconnected")
        reactor.stop()


if __name__ == '__main__':
    runner = ApplicationRunner(u'ws://192.168.49.80:55555', u'imu')
    runner.run(Component)
