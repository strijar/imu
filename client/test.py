#!/usr/bin/python3

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

class Component(ApplicationSession):

    def on_angle(self, i):
        print("Got angles: {}".format(i))

    def on_accel(self, i):
        print("Got accel: {}".format(i))

    def onJoin(self, details):
        print("session attached")
        self.subscribe(self.on_angle, u'angle')
#        self.subscribe(self.on_accel, u'accel')

    def onDisconnect(self):
        print("disconnected")
        reactor.stop()


if __name__ == '__main__':
    runner = ApplicationRunner(u'ws://localhost:55555', u'imu')
    runner.run(Component)
