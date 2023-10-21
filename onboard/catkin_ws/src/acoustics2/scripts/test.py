import sympy as sp

# Define the variables
x, y, D1, D2, t1, t2 = sp.symbols('x y D1 D2 t1 t2')

# Define the equations
eq1 = sp.sqrt(x**2 + y**2) - sp.sqrt(x**2 + (y - D1)**2) - t1
eq2 = sp.sqrt(x**2 + y**2) - sp.sqrt((x - D2)**2 + y**2) - t2

# Solve the equations
solutions = sp.solve((eq1, eq2), (x, y))

# Print the solutions
for sol in solutions:
    print(f"x = {sol[0]}")
    print(f"y = {sol[1]}\n")