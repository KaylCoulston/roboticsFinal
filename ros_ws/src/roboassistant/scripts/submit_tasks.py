#!/usr/bin/env python

import sqlite3
import datetime

conn = sqlite3.connect('tasks.db')
c = conn.cursor()

valid_locs = [69, 1, 12, 7, 200]
dict_locs={69:'Front Desk', 1:'Desk 1', 12:'Desk 2', 7:'Desk 3', 200:'Desk 4'}
print "LOCATIONS:"
print " 69  Front Desk"
print "  1  Desk 1"
print " 12  Desk 2"
print "  7  Desk 3"
print "200  Desk 4"
print

pickup_loc = -1
dropoff_loc = -1
while pickup_loc not in valid_locs:
    pickup_loc = int(input("Enter pickup location: "))

while dropoff_loc not in valid_locs:
    dropoff_loc = int(input("Enter dropoff location: "))
    if pickup_loc == dropoff_loc:
        print "ERROR: Pickup and dropoff at the same location."
        dropoff_loc = -1

now = datetime.datetime.now()
# Insert a row of data
insert_command = "INSERT INTO tasks VALUES ('%s',%d,%d)" % (str(now), pickup_loc, dropoff_loc)
print insert_command
c.execute(insert_command)

# Save (commit) the changes
conn.commit()
print "Your task request is now added to the queue!\n"

print "Tasks Queue:"
for row in c.execute('SELECT * FROM tasks ORDER BY date'):
    print str(row[0]), "from", dict_locs[row[1]], "to", dict_locs[row[2]]

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
conn.close()
