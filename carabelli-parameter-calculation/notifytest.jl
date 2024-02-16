using ResumableFunctions
using SimJulia

using Printf
macro printtime(env)
    esc(:(@printf "%10.6fs " now($env)))
end

@resumable function sensor(env::Environment, res::Resource)
    for i in 1:5
        @yield timeout(env, 0.1)
        @printtime env
        print("new measurement received")
        if(res.level == 0)
            println(" notifying controller")
            request(res)
        else
            println()
        end
    end
end

@resumable function controller(env::Environment, res::Resource)
    while true
        @yield release(res)
        @printtime env
        println("measurement acknowledged, doing stuff...")
        @yield timeout(env, 1.0)
    end
end

sim = Simulation()
res = Resource(sim)
@process sensor(sim, res)
@process controller(sim, res)

run(sim, 10)
