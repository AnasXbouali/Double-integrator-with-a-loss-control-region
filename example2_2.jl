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
    
    x1(0)  ==  0
    x2(0)  ==  3
    x1(tf) == 0.0
    x2(tf) == 0.0
    
    -1 ≤ u(t) ≤ 1
    -1 ≤ λ(t) ≤ 1
    
    q̇(t) == [x2(t),G(x1(t), x2(t))*u(t) + (1-G(x1(t), x2(t)))*λ(t), G(x1(t), x2(t))*v(t)]
    
    tf + ∫( ε*(v(t))^2 + (1-G(x1(t), x2(t)))*(u(t))^2 ) → min
end

sol1_ = solve(DI; init = (state = t -> [0.1, 0.1, 1], control = [1,0], variable = 30), grid_size=50)
sol2_ = solve(DI; init = sol1_, grid_size=100)
sol3_ = solve(DI; init = sol2_, grid_size=200)
sol4_ = solve(DI; init = sol3_, grid_size=300)
sol5_ = solve(DI; init = sol4_, grid_size=400)
sol6_ = solve(DI; init = sol5_, grid_size=500)
sol_  = solve(DI; init = sol6_, grid_size=1500)

tf_   = sol_.variable
y1_(t)= sol_.state(t)[1]
y2_(t)= sol_.state(t)[2]
p_ = plot(y1_, y2_, 0, tf_, color="blue", label=false)


# Create the plot
plt_ = plot(
    xlims=(-1,6), 
    ylims=(-3.5,3.5),
    xlabel = "x1",
    ylabel = "x2")

x_range = range(-20, 8, length=1000)
y_range = range(-20, 8, length=1000)

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

plot!(plt_, [-14, 6, 6, -14, -14], [-5, -5, 4, 4, -5], 
      fill=true, fillcolor=:lightgreen, fillalpha=0.4, 
      label=false, linecolor=nothing)

plot!(plt_, circle_x, circle_y, fill=true, fillcolor=:white, fillalpha=1, 
      label=false, linecolor=:black, linewidth=1)
plot!(plt_, circle_x, circle_y, fill=true, fillcolor=:red2, fillalpha=0.4, 
label=false, linecolor=:black, linewidth=1)

AA = plot!(plt_, y1_, y2_, 0, tf_, color="blue", lw=1.5, label="optimal trajectory")

diff_func_(t) = (y1_(t)-2)^2 + (y2_(t)+2)^2 - 1
t_cross_ = find_zero(diff_func_, (4, 5), Bisection())
t_cross2_ = find_zero(diff_func_, (5,6), Bisection())

u_(t) = sol_.control(t)[1]
λ0(t)= sol_.state(t)[3]
function control_(t)
    if t <= t_cross_
        return u_(t)
    elseif t_cross_ <= t <= t_cross2_
        return λ0(t)
    elseif t_cross2_ <= t 
        return u_(t)
    end
end
BB = plot(control_, 0, tf_, color="red", lw=1.5, label="optimal control")

q1_(t)= sol_.costate(t)[1]
q2_(t)= sol_.costate(t)[2]


plot(q1_, 0,tf_, color="purple4", lw=1.5, label="costate p1")
CC = plot!(q2_, 0,tf_, color="mediumorchid1", lw=1.5, label="costate p2")
plot(AA, BB, CC, layout=(1, 3), size=(1600,600))
savefig("plott2.pdf")
