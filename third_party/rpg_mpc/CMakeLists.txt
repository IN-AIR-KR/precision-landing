# Source layout from uzh-rpg/rpg_mpc. This target builds without catkin.
set(codegen model/quadrotor_mpc_codegen)
set(qpoases externals/qpoases)
add_library(rpg_mpc_solver STATIC
    ${codegen}/acado_qpoases_interface.cpp
    ${codegen}/acado_integrator.c
    ${codegen}/acado_solver.c
    ${codegen}/acado_auxiliary_functions.c
    ${qpoases}/SRC/Bounds.cpp
    ${qpoases}/SRC/Constraints.cpp
    ${qpoases}/SRC/CyclingManager.cpp
    ${qpoases}/SRC/Indexlist.cpp
    ${qpoases}/SRC/MessageHandling.cpp
    ${qpoases}/SRC/QProblem.cpp
    ${qpoases}/SRC/QProblemB.cpp
    ${qpoases}/SRC/SubjectTo.cpp
    ${qpoases}/SRC/Utils.cpp
)
target_include_directories(rpg_mpc_solver SYSTEM PUBLIC
    ${codegen} ${qpoases} ${qpoases}/INCLUDE ${qpoases}/SRC)
# The generated code and bundled qpOASES predate current warning policies.
target_compile_options(rpg_mpc_solver PRIVATE -Wno-error)
