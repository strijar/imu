#!/usr/bin/python3

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

class Component(ApplicationSession):

    def on_mag(self, i):
        print("Got mag: {}".format(i))
        with open("mag.txt", "a") as f:
           raw = i['raw']
           f.write("%i\t%i\t%i\n" % (raw["x"], raw["y"], raw["z"]))
           f.close()

        with open("mag_cal.txt", "a") as f:
           cal = i['cal']
           f.write("%.1f\t%.1f\t%.1f\n" % (cal["x"], cal["y"], cal["z"]))
           f.close()

    def onJoin(self, details):
        print("session attached")
        self.subscribe(self.on_mag, u'calMag')

    def onDisconnect(self):
        print("disconnected")
        reactor.stop()


if __name__ == '__main__':
    runner = ApplicationRunner(u'ws://192.168.49.94:55555', u'imu')
    runner.run(Component)
