import testApp

numRobots = 3
for i in range(numRobots):
    app = testApp.TestApp(i)
    print(app)
    app.start()