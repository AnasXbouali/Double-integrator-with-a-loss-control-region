using OptimalControl
using NLPModelsIpopt
using Plots
using Plots.PlotMeasures
using Roots

function G(x, y, k=100)
    return 1 / (1 + exp(-k * ((x-2)^2 + (y+2)^2 - 1)))
end

ε = 1e-4

DI = @def begin

    tf ∈ R, variable
    t ∈ [0., tf], time
    q = [x1, x2, λ] ∈ R^3, state
    ω = [u,v] ∈ R^2, control

    tf ≥ 0
    
    x1(0)  ==  2
    x2(0)  ==  2
    x1(tf) == 0.0
    x2(tf) == 0.0
    
    -1 ≤ u(t) ≤ 1
    -1 ≤ λ(t) ≤ 1
    
    q̇(t) == [x2(t),G(x1(t), x2(t))*u(t) + (1-G(x1(t), x2(t)))*λ(t), G(x1(t), x2(t))*v(t)]
    
    tf + ∫( ε*(v(t))^2 + (1-G(x1(t), x2(t)))*(u(t))^2 ) → min
end

sol1 = solve(DI; init = (state = t -> [0.1, 0.1, 0.1], control = [0.1,0.1], variable = 17), grid_size=50, print_level=4)
objective(sol1)

sol2 = solve(DI; init = sol1, grid_size=200,  print_level=4)
objective(sol2)

sol3 = solve(DI; init = sol2, grid_size=400,print_level=4)
objective(sol3)

sol4 = solve(DI; init = sol3, grid_size=800, print_level=4)
objective(sol4)

sol = solve(DI; init = sol4, grid_size=1600, print_level=4)
objective(sol)

tf   = sol.variable
y1(t)= sol.state(t)[1]
y2(t)= sol.state(t)[2]
p = plot(y1, y2, 0, tf, color="blue", label=false)


# Create the plot
plt_ = plot(
    xlims=(-1,5), 
    ylims=(-3.5,2.5),
    xlabel = "x1",
    ylabel = "x2")

x_range = range(-20, 5, length=1000)
y_range = range(-20, 5, length=1000)

xs = repeat(x_range, 1, length(y_range))
ys = repeat(y_range', length(x_range), 1)

function in_circle(x, y, center_x, center_y, radius)
    return (x - center_x)^2 + (y - center_y)^2 ≤ radius^2
end

circle_center = (2, -2)
circle_radius = 1

θ = range(0, 2π, length=100)
circle_x = circle_center[1] .+ circle_radius .* cos.(θ)
circle_y = circle_center[2] .+ circle_radius .* sin.(θ)

plot!(plt_, [-14, 5, 5, -14, -14], [-5, -5, 4, 4, -5], 
      fill=true, fillcolor=:lightgreen, fillalpha=0.4, 
      label=false, linecolor=nothing)

plot!(plt_, circle_x, circle_y, fill=true, fillcolor=:white, fillalpha=1, 
      label=false, linecolor=:black, linewidth=1)
plot!(plt_, circle_x, circle_y, fill=true, fillcolor=:red2, fillalpha=0.4, 
label=false, linecolor=:black, linewidth=1)

A = plot!(plt_, y1, y2, 0, tf, color="blue", lw=1.5, label="optimal trajectory")

diff_func_(t) = (y1(t)-2)^2 + (y2(t)+2)^2 - 1
t_cross_ = find_zero(diff_func_, (3, 4), Bisection())
t_cross2_ = find_zero(diff_func_, (4,5), Bisection())

u(t) = sol.control(t)[1]
λ0(t)= sol.state(t)[3]
function control(t)
    if t <= t_cross_
        return u(t)
    elseif t_cross_ <= t <= t_cross2_
        return λ0(t)
    elseif t_cross2_ <= t 
        return u(t)
    end
end
B = plot(control, 0, tf, color="red", lw=1.5, label="optimal control")

q1(t)= sol.costate(t)[1]
q2(t)= sol.costate(t)[2]


plot(q1, 0,tf, color="purple4", lw=1.5, label="costate p1")
C = plot!(q2, 0,tf, color="mediumorchid1", lw=1.5, label="costate p2")
plot(A, B, C, layout=(1, 3), size=(1600,500))
savefig("plott1.pdf")
