#!/usr/bin/python3

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner
from autobahn import wamp

class Component(ApplicationSession):

    @wamp.subscribe(u'mag.raw')
    def on_mag(self, i):
        print("Got mag: {}".format(i))

    @wamp.subscribe(u'accel.raw')
    def on_accel(self, i):
        print("Got accel: {}".format(i))

    @wamp.subscribe(u'gyro.raw')
    def on_angle(self, i):
        print("Got gyro: {}".format(i))

    @inlineCallbacks
    def onJoin(self, details):
        print("session attached")

        res = yield self.call(u'SetFreq', freq=10)
        print("Answer: {}".format(res))

        results = yield self.subscribe(self)

    def onDisconnect(self):
        print("disconnected")
        reactor.stop()


if __name__ == '__main__':
    runner = ApplicationRunner(u'ws://192.168.49.94:55555', u'imu')
    runner.run(Component)
