import m5
from m5.objects import *

# All gem5 instances require a Root object.
root = Root(full_system = False)

# Instantiate the HelloObject you created in Python.
# This is done by calling the Python “constructor”.
# Make sure the object you instantiated a child of the root object.
# Only SimObjects that are children of the Root object are instantiated in C++.
root.hello = HelloObject()
root.hello.time_to_wait = '2us'

# instantiate all of the objects we've created
m5.instantiate()

print("Beginning simulation!")
exit_event = m5.simulate()
print('Exiting @ tick {} because {}'
      .format(m5.curTick(), exit_event.getCause()))

