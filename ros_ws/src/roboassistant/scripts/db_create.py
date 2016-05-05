#!/usr/bin/env python

import sqlite3

conn = sqlite3.connect('tasks.db')

c = conn.cursor()

# Create table
c.execute('''CREATE TABLE tasks
             (date text, pickup_id int, drop_id int)''')


# Save (commit) the changes
conn.commit()

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
conn.close()
