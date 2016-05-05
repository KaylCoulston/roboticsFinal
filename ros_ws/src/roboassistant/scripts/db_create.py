#!/usr/bin/env python

import sqlite3

conn = sqlite3.connect('tasks.db')

c = conn.cursor()

# Create table
c.execute('''CREATE TABLE tasks
             (id int primary key not null, date_time text not null, pickup_id int not null, drop_id int not null)''')


# Save (commit) the changes
conn.commit()

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
conn.close()
