using OptimalControl
using NLPModelsIpopt
using Plots
using Plots.PlotMeasures
using Roots

function F(x, y, k=80)
    return 1 / (1 + exp(-k * (y - x)))
end

ε  = 1e-4 

# First example: intial condition (2,0)

DI = @def begin

    tf ∈ R, variable
    t ∈ [0., tf], time
    q = [x1, x2, λ] ∈ R^3, state
    ω = [u,v] ∈ R^2, control

    tf ≥ 0
    
    x1(0)  ==  2
    x2(0)  ==  0
    x1(tf) == 0.0
    x2(tf) == 0.0

    -1 ≤ u(t) ≤ 1
    -1 ≤ λ(t) ≤ 1
    
    q̇(t) == [x2(t),F(x1(t), x2(t))*u(t) + (1-F(x1(t), x2(t)))*λ(t), F(x1(t), x2(t))*v(t)]
    
    tf + ∫( ε*(v(t))^2 + (1-F(x1(t), x2(t)))*(u(t))^2 ) → min
end

sol1 = solve(DI; init = (state = t -> [0.1, 0.1, 1], control = [1,0], variable = 30), grid_size=50)
sol2 = solve(DI; init = sol1, grid_size=100)
sol3 = solve(DI; init = sol2, grid_size=200)
sol4 = solve(DI; init = sol3, grid_size=300)
sol5 = solve(DI; init = sol4, grid_size=400)
sol6 = solve(DI; init = sol5, grid_size=500)
sol  = solve(DI; init = sol6, grid_size=1000, print_level=4)

# Extract solution
tf   = sol.variable
y1(t)= sol.state(t)[1]
y2(t)= sol.state(t)[2]
plt = plot(y1, y2, 0, tf, color="blue", label=false)

# Plot states
x = range(-2.2, 2.2, length=200)
y = range(-2.2, 2.2, length=200)
Z = [y > x for x in x, y in y]
heatmap!(plt, x, y, Z, color=[:lightgreen, :red2], fillalpha=0.4, colorbar=false)
plot!(
    x -> x,  
    color = :black,
    label = false,
    linewidth = 2)
A = plot!(y1, y2, 0, tf, color="blue", lw=1.5, label="optimal trajectory")

# Plot control
u(t)= sol.control(t)[1]
λ(t)= sol.state(t)[3]
diff_func(t) = y2(t) - y1(t)
t_cross = find_zero(diff_func, (4, 6), Bisection())
function control(t)
    if t <= t_cross
        return λ(t)
    elseif t_cross <= t
        return u(t)
    end 
end
B = plot(control, 0,tf,color="red", lw=1.5, label="optimal control")

# Plot costates
q1(t)= sol.costate(t)[1]
q2(t)= sol.costate(t)[2]
plot(q1, 0,tf, color="purple4", lw=1.5, label="costate p1")
C = plot!(q2, 0,tf, color="mediumorchid1", lw=1.5, label="costate p2")

plot(A,B,C,layout=(1, 3), size=(1600,600))