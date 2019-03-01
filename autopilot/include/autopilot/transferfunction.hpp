#ifndef TRANSFERFUNCTION_HPP
#define TRANSFERFUNCTION_HPP

#include <Eigen/Core>

template<typename T,int N,int D>
class Transferfunction
{
public:
    Transferfunction(Eigen::Matrix<T,N,1> num,Eigen::Matrix<T,D,1> den);
    virtual ~Transferfunction();

private:
    Eigen::Matrix<T,N,1> m_num;
    Eigen::Matrix<T,D,1> m_den;
    Eigen::Matrix<T,D-1,1> m_state;
    Eigen::Matrix<T,N-1,N-1> m_A;
    Eigen::Matrix<T,N-1,1> m_B;
    Eigen::Matrix<T,1,N-1> m_C;
    
};

#endif /* TRANSFERFUNCTION_H */
