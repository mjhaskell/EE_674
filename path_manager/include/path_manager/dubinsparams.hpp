#ifndef DUBINSPARAMS_HPP
#define DUBINSPARAMS_HPP

#include <Eigen/Core>

class DubinsParams
{
public:
    DubinsParams()
    {}

    bool update(const Eigen::Vector3d& ps, const Eigen::Vector3d& pe,
                double chis, double chie, double R)
    {
        double ell{(ps-pe).norm()};
        if (ell < 2*R)
            return false;

        Eigen::Vector3d direction_s, direction_e;
        direction_s << cos(chis), sin(chis), 0;
        direction_e << cos(chie), sin(chie), 0;

        Eigen::Vector3d c_rs, c_ls, c_re, c_le;
        Eigen::Matrix3d Rz;
        rotz(PI/2.0, Rz);
        c_rs = ps + R*Rz*direction_s;
        c_re = pe + R*Rz*direction_e;
        rotz(-PI/2.0, Rz);
        c_ls = ps + R*Rz*direction_s;
        c_le = pe + R*Rz*direction_e;

        double L1, L2, L3, L4, L;
        double theta, theta2, sq_rt;
        // compute L1
        theta = atan2(c_re(1)-c_rs(1), c_re(0)-c_rs(0));
        L1 = (c_rs-c_re).norm() +
              R*mod(2*PI+mod(theta-PI/2.0)-mod(chis-PI/2.0)) +
              R*mod(2*PI+mod(chie-PI/2.0)-mod(theta-PI/2.0));

        // compute L1
        theta = atan2(c_le(1)-c_rs(1), c_le(0)-c_rs(0));
        ell = (c_rs-c_le).norm();
        sq_rt = sqrt(ell*ell - 4*R*R);
        theta2 = theta - PI/2.0 + asin(2*R/ell);
        L2 = sq_rt + R*mod(2*PI+mod(theta-theta2)-mod(chis-PI/2.0)) +
                     R*mod(2*PI+mod(theta2+PI)-mod(chie+PI/2.0));

        // compute L3
        theta = atan2(c_re(1)-c_ls(1), c_re(0)-c_ls(0));
        ell = (c_ls-c_re).norm();
        sq_rt = sqrt(ell*ell - 4*R*R);
        theta2 = acos(2*R/ell);
        L3 = sq_rt + R*mod(2*PI+mod(chis+PI/2.0)-mod(theta+theta2)) +
                     R*mod(2*PI+mod(chie-PI/2)-mod(theta+theta2-PI));

        // compute L4
        theta = atan2(c_le(1)-c_ls(1), c_le(0)-c_ls(0));
        L4 =  (c_ls-c_le).norm() +
               R*mod(2*PI+mod(chis+PI/2)-mod(theta+PI/2.0)) +
               R*mod(2*PI+mod(theta+PI/2.0)-mod(chie+PI/2.0));

        L = std::min(std::min(L1,L2),std::min(L3,L4));

        rotz(chie, Rz);
        m_r3 = pe;
        m_n3 = Rz.col(0);

        if (L == L1)
        {
            m_center_s = c_rs;
            m_center_e = c_re;
            m_dir_s = 1;
            m_dir_e = 1;
            m_n1 = m_center_e - m_center_s;
            ell = m_n1.norm();
            m_n1 /= m_n1.norm();
            rotz(-PI/2.0,Rz);
            m_r1 = m_center_s + R*Rz*m_n1;
            m_r2 = m_center_e + R*Rz*m_n1;
        }
        else if (L == L2)
        {
            m_center_s = c_rs;
            m_center_e = c_le;
            m_dir_s = 1;
            m_dir_e = -1;
            ell = (m_center_e-m_center_s).norm();
            theta = atan2(m_center_e(1)-m_center_s(1), m_center_e(0)-m_center_s(0));
            theta2 = theta - PI/2.0 + asin(2*R/ell);
            rotz(theta2+PI/2.0, Rz);
            m_n1 = Rz.col(0);
            rotz(theta2, Rz);
            m_r1 = m_center_s + R*Rz.col(0);
            rotz(theta2+PI, Rz);
            m_r2 = m_center_e + R*Rz.col(0);
        }
        else if (L == L3)
        {
            m_center_s = c_ls;
            m_center_e = c_re;
            m_dir_s = -1;
            m_dir_e = 1;
            ell = (m_center_e-m_center_s).norm();
            theta = atan2(m_center_e(1)-m_center_s(1), m_center_e(0)-m_center_s(0));
            theta2 = acos(2*R/ell);
            rotz(theta+theta2-PI/2.0, Rz);
            m_n1 = Rz.col(0);
            rotz(theta+theta2, Rz);
            m_r1 = m_center_s + R*Rz.col(0);
            rotz(theta+theta2-PI, Rz);
            m_r2 = m_center_e + R*Rz.col(0);
        }
        else
        {
            m_center_s = c_ls;
            m_center_e = c_le;
            m_dir_s = -1;
            m_dir_e = -1;
            m_n1 = m_center_e - m_center_s;
            ell = m_n1.norm();
            m_n1 /= m_n1.norm();
            rotz(PI/2.0, Rz);
            m_r1 = m_center_s + R*Rz*m_n1;
            m_r2 = m_center_e + R*Rz*m_n1;
        }

        m_radius = R;
        m_length = ell;
        m_chi_e = chie;
        m_chi_s = chis;
        m_p_s = ps;
        m_p_e = pe;

        return true;
    }

    void rotz(double theta, Eigen::Matrix3d& Rz)
    {
        Rz << cos(theta), -sin(theta), 0,
              sin(theta),  cos(theta), 0,
              0,           0,          1;
    }

    double mod(double x)
    {
        return x - int(x/(2*PI)) * 2*PI;
    }

    const double PI{3.14159};

    Eigen::Vector3d m_p_s{0,0,0}, m_p_e{0,0,0};
    Eigen::Vector3d m_center_s{0,0,0}, m_center_e{0,0,0};
    Eigen::Vector3d m_r1{0,0,0}, m_r2{0,0,0}, m_r3{0,0,0};
    Eigen::Vector3d m_n1{0,0,0}, m_n3{0,0,0};
    double m_chi_s{INFINITY}, m_chi_e{INFINITY};
    double m_radius{INFINITY}, m_length{INFINITY};
    int m_dir_s{0}, m_dir_e{0};
};

#endif // DUBINSPARAMS_HPP
