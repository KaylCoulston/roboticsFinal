#!/usr/bin/env python

import sqlite3
import datetime

fp = open('id_num.txt', 'r')
id_num = fp.readline()
id_num = int(id_num.rstrip()) + 1
fp.close()
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
insert_command = "INSERT INTO tasks VALUES (%d,'%s',%d,%d)" % (id_num, str(now), pickup_loc, dropoff_loc)
c.execute(insert_command)
# update id num
fp = open('id_num.txt', 'w')
fp.write(str(id_num))
fp.close()

# Save (commit) the changes
conn.commit()
print "Your task request is now added to the queue!\n"

# c.execute('SELECT Count(*) FROM tasks')
# print c.fetchone()


print "Tasks Queue:"
for row in c.execute('SELECT * FROM tasks ORDER BY date_time'):
    print row[0], str(row[1]), "from", dict_locs[row[2]], "to", dict_locs[row[3]]

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
conn.close()
