# Generated solver and qpOASES from FSC-Lab/fsc_aggressive_ibvs.
set(codegen model/pose_track_mpc)
set(qpoases externals/qpoases)
add_library(fsc_ibvs_solver STATIC
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
target_include_directories(fsc_ibvs_solver SYSTEM PUBLIC
    ${codegen} ${qpoases} ${qpoases}/INCLUDE ${qpoases}/SRC)
# The generated code and bundled qpOASES predate current warning policies.
target_compile_options(fsc_ibvs_solver PRIVATE -Wno-error)
