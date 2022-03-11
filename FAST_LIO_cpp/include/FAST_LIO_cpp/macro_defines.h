#pragma once

#define FAST_LIO_VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define FAST_LIO_MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define FAST_LIO_DEBUG_FILE_DIR(name)     (std::string(std::string(FAST_LIO_ROOT_DIR) + "Log/"+ name))
#define FAST_LIO_SKEW_SYM_MATRIX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

// unused:
// #define MAX_MEAS_DIM        (10000)
// #define USE_IKFOM
// #define CUBE_LEN  (6.0)


// unused
// TODO: refactor into static functions of Common 
// #define CONSTRAIN(v,min,max)     ((v>min)?((v<max)?v:max):min)
// #define ARRAY_FROM_EIGEN(mat)    mat.data(), mat.data() + mat.rows() * mat.cols()
// #define STD_VEC_FROM_EIGEN(mat)  std::vector<decltype(mat)::Scalar> (mat.data(), mat.data() + mat.rows() * mat.cols())
