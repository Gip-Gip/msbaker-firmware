import math as m

start_x = 9.8*0.687
start_y = 9.8*0.526
start_z = 9.8*0.499

tx = m.atan2(start_y, start_z);
ty = m.atan2(-start_x, start_z);
tz = m.atan2(start_x, start_y);

x = start_x
y = start_y
z = start_z

# Rotate X with an inverted rotation matrix
n_x = x
n_y = y * m.cos(tx) + z * -m.sin(tx)
n_z = y * m.sin(tx) + z * m.cos(tx)

x = n_x
y = n_y
z = n_z

# Rotate the Y with an inverted rotation matrix
n_x = x * m.cos(ty) + z * m.sin(ty)
n_y = y
n_z = x * -m.sin(ty) + z * m.cos(ty)

x = n_x
y = n_y
z = n_z

# Rotate the Z with an inverted rotation matrix
n_x = x * m.cos(tz) + y * -m.sin(tz)
n_y = x * m.sin(tz) + y * m.cos(tz)
n_z = z

x = n_x
y = n_y
z = n_z

end_x = x
end_y = y
end_z = z

print(str(tx) + ", " + str(ty) + ", " + str(tz))
print(str(end_x) + ", " + str(end_y) + ", " + str(end_z))
