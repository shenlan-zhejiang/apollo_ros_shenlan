#include "mpc.h"
 
using namespace std;
namespace apollo{
namespace shenlan {
void MPC::init()
{
    du_th = 0.1;//-1.0;
    dt = 0.01;//-1.0;
    wheel_base = 0.322;//-1.0;
    max_iter = 3;//-1;
    T = -1;
    in_test = true;//false;
    max_steer = 0.4055;//-1.0;
    max_dsteer = 0.1898;//-1.0;
    max_speed = 0.5;//-1.0;
    min_speed = -0.5;//-1.0;
    max_accel = 0.2;//-1.0;
    control_a = true;//false;
    feed_forward = false;
    std::vector<double>  Q = {10, 10, 2.5, 0.5};
    std::vector<double> R ={0.01, 0.01};
    std::vector<double> Rd = {0.01, 1.0};
    test_traj = "xxx";

    has_odom = false;
    receive_traj_ = false;
    max_csteer = max_dsteer * dt;
    max_cv = max_accel * dt;
    xref = Eigen::Matrix<double, 4, 240>::Zero(4, 240);
    last_output = output = dref = Eigen::Matrix<double, 2, 240>::Zero(2, 240);
}

void MPC::getLinearModel(const MPCState& s, double delta)
{
    if (control_a)
    {
        A = Eigen::Matrix4d::Identity();
        A(0, 2) = dt * cos(s.theta);
        A(1, 2) = dt * sin(s.theta);
        A(0, 3) = -s.v * A(1, 2);
        A(1, 3) = s.v * A(0, 2);
        A(3, 2) = dt * tan(delta) / wheel_base;

        B = Eigen::Matrix<double, 4, 2>::Zero();
        B(2, 0) = dt;
        B(3, 1) = dt * s.v / (wheel_base * pow(cos(delta), 2));

        C = Eigen::Vector4d::Zero();
        C(0) = -A(0, 3) * s.theta;
        C(1) = -A(1, 3) * s.theta;
        C(3) = -B(3, 1) * delta;
    }
    else
    {
        B = Eigen::Matrix<double, 3, 2>::Zero();
        B(0, 0) = cos(s.theta) * dt;
        B(1, 0) = sin(s.theta) * dt;
        B(2, 0) = dt * tan(delta) / wheel_base;
        B(2, 1) = dt * s.v / (wheel_base * pow(cos(delta), 2));

        A = Eigen::Matrix3d::Identity();
        A(0, 2) = -B(1, 0) * s.v;
        A(1, 2) = B(0, 0) * s.v;

        C = Eigen::Vector3d::Zero();
        C(0) = -A(0, 2) * s.theta; 
        C(1) = -A(1, 2) * s.theta; 
        C(2) = -B(2, 1) * delta; 
    }
}

void MPC::stateTrans(MPCState& s, double a, double delta)
{
    if (control_a)
    {
        if (delta >= max_steer)
        {
            delta = max_steer;
        }else if (delta<= - max_steer)
        {
            delta = -max_steer;
        }

        s.x = s.x + s.v * cos(s.theta) * dt;
        s.y = s.y + s.v * sin(s.theta) * dt;
        s.theta = s.theta + s.v / wheel_base * tan(delta) * dt;
        s.v = s.v + a * dt;

        if (s.v >= max_speed)
        {
            s.v = max_speed;
        }else if (s.v <= min_speed)
        {
            s.v = min_speed;
        }
    }
    else
    {
        if (delta >= max_steer)
        {
            delta = max_steer;
        }else if (delta<= - max_steer)
        {
            delta = -max_steer;
        }
        if (s.v >= max_speed)
        {
            s.v = max_speed;
        }else if (s.v<= min_speed)
        {
            s.v = min_speed;
        }

        s.x = s.x + a * cos(s.theta) * dt;
        s.y = s.y + a * sin(s.theta) * dt;
        s.theta = s.theta + a / wheel_base * tan(delta) * dt;
        s.v = a;
    }
}

void MPC::predictMotion(void)
{
    xbar[0] = now_state;

    MPCState temp = now_state;
    for (int i=1; i<T+1; i++)
    {
        stateTrans(temp, output(0, i-1), output(1, i-1));
        xbar[i] = temp;
    }
}

void MPC::predictMotion(MPCState* b)
{
    b[0] = xbar[0];

    Eigen::MatrixXd Ax;
    Eigen::MatrixXd Bx;
    Eigen::MatrixXd Cx;
    Eigen::MatrixXd xnext;
    MPCState temp = xbar[0];
    for (int i=1; i<T+1; i++)
    {
        Ax = Eigen::Matrix4d::Identity();
        Ax(0, 2) = dt * cos(xbar[i-1].theta);
        Ax(1, 2) = dt * sin(xbar[i-1].theta);
        Ax(0, 3) = -xbar[i-1].v * Ax(1, 2);
        Ax(1, 3) = xbar[i-1].v * Ax(0, 2);
        Ax(3, 2) = 0.0;

        Bx = Eigen::Matrix<double, 4, 2>::Zero();
        Bx(2, 0) = dt;
        Bx(3, 1) = dt * xbar[i-1].v / wheel_base;

        Cx = Eigen::Vector4d::Zero();
        Cx(0) = -Ax(0, 3) * xbar[i-1].theta;
        Cx(1) = -Ax(1, 3) * xbar[i-1].theta;
        Cx(3) = 0.0;
        
        xnext = Ax*Eigen::Vector4d(temp.x, temp.y, temp.v, temp.theta) + Bx*Eigen::Vector2d(output(0, i-1), output(1, i-1)) + Cx;
        temp.x = xnext(0);
        temp.y = xnext(1);
        temp.v = xnext(2);
        temp.theta = xnext(3);
        // stateTrans(temp, output(0, i-1), output(1, i-1));
        b[i] = temp;
        // temp = xbar[i];
    }
}

void MPC::solveMPCA(void)
{
    static int debug_dt = 0;

    const int dimx = 4 * T;
    const int dimu = 2 * T;
    const int nx = dimx + dimu;

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // first-order
    for (int i=0, j=0; i<dimx; i+=4, j++)
    {
        gradient[i] = -2 * Q[0] * xref(0, j);
        gradient[i+1] = -2 * Q[1] * xref(1, j);
        gradient[i+2] = -2 * Q[2] * xref(2, j);
        gradient[i+3] = -2 * Q[3] * xref(3, j);
    }
    for (int i=dimx, j=0; i<dimu; i+=2, j++)
    {
        gradient[i] = -2 * R[0] * dref(0, j);
        gradient[i+1] = -2 * R[1] * dref(1, j);
    }

    // second-order
    const int nnzQ = nx + dimu - 2;
    int irowQ[nnzQ];
    int jcolQ[nnzQ];
    double dQ[nnzQ];
    for (int i=0; i<nx; i++)
    {
        irowQ[i] = jcolQ[i] = i;
    }
    for (int i=nx; i<nnzQ; i++)
    {
        irowQ[i] = i - dimu + 2;
        jcolQ[i] = i - dimu;
    }
    for (int i=0; i<dimx; i+=4)
    {
        dQ[i] = Q[0] * 2.0;
        dQ[i+1] = Q[1] * 2.0;
        dQ[i+2] = Q[2] * 2.0;
        dQ[i+3] = Q[3] * 2.0;
    }
    dQ[dimx] = dQ[nx-2] = (R[0] + Rd[0]) * 2.0;
    dQ[dimx + 1] = dQ[nx-1] = (R[1] + Rd[1]) * 2.0;
    for (int i=dimx+2; i<nx-2; i+=2)
    {
        dQ[i] = 2 * (R[0] + 2 * Rd[0]);
        dQ[i+1] = 2 * (R[1] + 2 * Rd[1]);
    }
    for (int i=nx; i<nnzQ; i+=2)
    {
        dQ[i] = -Rd[0] * 2.0;
        dQ[i+1] = -Rd[1] * 2.0;
    }
    hessian.resize(nx, nx);
    Eigen::MatrixXd QQ(nx, nx);
    for (int i=0; i<nx; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
    }
    for (int i=nx; i<nnzQ; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
        hessian.insert(jcolQ[i], irowQ[i]) = dQ[i];
    }

    // equality constraints
    MPCState temp = xbar[0];
    getLinearModel(temp, dref(1, 0));
    int my = dimx;
    double b[my];
    const int nnzA = 15 * T - 9;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];
    Eigen::Vector4d temp_vec(temp.x, temp.y, temp.v, temp.theta);
    Eigen::Vector4d temp_b = A*temp_vec + C;
    
    for (int i=0; i<dimx; i++)
    {
        irowA[i] = jcolA[i] = i;
        dA[i] = 1;
    }
    b[0] = temp_b[0];
    b[1] = temp_b[1];
    b[2] = temp_b[2];
    b[3] = temp_b[3];
    irowA[dimx] = 2;
    jcolA[dimx] = dimx;
    dA[dimx] = -B(2, 0);
    irowA[dimx+1] = 3;
    jcolA[dimx+1] = dimx+1;
    dA[dimx+1] = -B(3, 1);
    int ABidx = 11*T - 11;
    int ABbegin = dimx+2;
    for (int i=0, j=1; i<ABidx; i+=11, j++)
    {
        getLinearModel(xbar[j], dref(1, j));
        for (int k=0; k<4; k++)
        {
            b[4*j+k] = C[k];
            irowA[ABbegin + i + k] = 4*j + k;
            jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 4;
            dA[ABbegin + i + k] = -A(k, k);
        }
        irowA[ABbegin + i + 4] = 4*j;
        jcolA[ABbegin + i + 4] = 4*j - 2;
        dA[ABbegin + i + 4] = -A(0, 2);
        
        irowA[ABbegin + i + 5] = 4*j;
        jcolA[ABbegin + i + 5] = 4*j - 1;
        dA[ABbegin + i + 5] = -A(0, 3);

        irowA[ABbegin + i + 6] = 4*j + 1;
        jcolA[ABbegin + i + 6] = 4*j - 2;
        dA[ABbegin + i + 6] = -A(1, 2);

        irowA[ABbegin + i + 7] = 4*j + 1;
        jcolA[ABbegin + i + 7] = 4*j - 1;
        dA[ABbegin + i + 7] = -A(1, 3);
        
        irowA[ABbegin + i + 8] = 4*j + 3;
        jcolA[ABbegin + i + 8] = 4*j - 2;
        dA[ABbegin + i + 8] = -A(2, 3);
        
        irowA[ABbegin + i + 9] = 4*j +2;
        jcolA[ABbegin + i + 9] = dimx + 2*j;
        dA[ABbegin + i + 9] = -B(2, 0);
        
        irowA[ABbegin + i + 10] = 4*j + 3;
        jcolA[ABbegin + i + 10] = dimx + 2*j + 1;
        dA[ABbegin + i + 10] = -B(3, 1);
    }

    // iequality constraints
    const int mz  = T - 1;
    const int nnzC = dimu - 2;
    int   irowC[nnzC];
    int   jcolC[nnzC];
    double   dC[nnzC];
    for (int i=0, j=0; i<mz; i++, j+=2)
    {
        irowC[j] = i;
        irowC[j+1] = i;
        jcolC[j] = dimx + 1 + j;
        jcolC[j+1] = jcolC[j] +2;
        dC[j] = -1.0;
        dC[j+1] = 1.0;
    }

    // xlimits and all
    int mx = 3*T;
    int nc = mx+my+mz;
    lowerBound.resize(nc);
    upperBound.resize(nc);
    linearMatrix.resize(nc, nx);
    for (int i=0, j=0, k=0; i<mx; i+=3, j+=4, k+=2)
    {
        lowerBound[i] = min_speed;
        lowerBound[i+1] = -max_accel;
        lowerBound[i+2] = -max_steer;
        upperBound[i] = max_speed;
        upperBound[i+1] = max_accel;
        upperBound[i+2] = max_steer;
        linearMatrix.insert(i, j+2) = 1;
        linearMatrix.insert(i+1, dimx+k) = 1;
        linearMatrix.insert(i+2, dimx+k+1) = 1;
    }
    for (int i=0; i<nnzA; i++)
    {
        linearMatrix.insert(irowA[i]+mx, jcolA[i]) = dA[i];
    }
    for (int i=0; i<my; i++)
    {
        lowerBound[mx+i] = upperBound[mx+i] = b[i];
    }
    for (int i=0; i<nnzC; i++)
    {
        linearMatrix.insert(irowC[i]+mx+my, jcolC[i]) = dC[i];
    }
    for (int i=0; i<mz; i++)
    {
        lowerBound[mx+my+i] = -max_csteer;
        upperBound[mx+my+i] = max_csteer;
    }
    return ;

    // instantiate the solver
    OsqpEigen::Solver solver;
    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-6);
    solver.settings()->setMaxIteration(30000);
    solver.settings()->setRelativeTolerance(1e-6);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);
    if(!solver.data()->setHessianMatrix(hessian)) return;
    if(!solver.data()->setGradient(gradient)) return;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return;
    if(!solver.data()->setLowerBound(lowerBound)) return;
    if(!solver.data()->setUpperBound(upperBound)) return;

    // instantiate the solver
    if(!solver.initSolver()) return;

    // controller input and QPSolution vector
    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;

    // get the controller input
    QPSolution = solver.getSolution();
    if (debug_dt++>100)
    {
        // ROS_INFO("Solution: a0=%f     delta0=%f", QPSolution[dimx], QPSolution[dimx+1]);
        debug_dt = 0;
    }
    for (int i=0; i<dimu; i+=2)
    {
        output(0, i) = QPSolution[dimx+i];
        output(1, i) = QPSolution[dimx+i+1];
    }
    // for (int i=0, j=0; i<dimu; i+=2, j++)
    // {
    //     output(0, j) = QPSolution[dimx+i];
    //     output(1, j) = QPSolution[dimx+i+1];
    // }
}

void MPC::solveMPCV(void)
{
    return;
    const int dimx = 3 * T;
    const int dimu = 2 * T;
    const int nx = dimx + dimu;

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // first-order
    for (int i=0, j=0, k=0; i<dimx; i+=3, j++, k+=2)
    {
        gradient[i] = -2 * Q[0] * xref(0, j);
        gradient[i+1] = -2 * Q[1] * xref(1, j);
        gradient[i+2] = -2 * Q[3] * xref(3, j);
        gradient[dimx+k] = -2 * Q[2] * dref(0, j);
    }

    // second-order
    const int nnzQ = nx + dimu - 2;
    int irowQ[nnzQ];
    int jcolQ[nnzQ];
    double dQ[nnzQ];
    for (int i=0; i<nx; i++)
    {
        irowQ[i] = jcolQ[i] = i;
    }
    for (int i=nx; i<nnzQ; i++)
    {
        irowQ[i] = i - dimu + 2;
        jcolQ[i] = i - dimu;
    }
    for (int i=0; i<dimx; i+=3)
    {
        dQ[i] = Q[0] * 2.0;
        dQ[i+1] = Q[1] * 2.0;
        dQ[i+2] = Q[3] * 2.0;
    }
    dQ[dimx] = dQ[nx-2] = (R[0] + Rd[0] + Q[2]) * 2.0;
    dQ[dimx + 1] = dQ[nx-1] = (R[1] + Rd[1]) * 2.0;
    for (int i=dimx+2; i<nx-2; i+=2)
    {
        dQ[i] = 2 * (R[0] + 2 * Rd[0] + Q[2]);
        dQ[i+1] = 2 * (R[1] + 2 * Rd[1]);
    }
    for (int i=nx; i<nnzQ; i+=2)
    {
        dQ[i] = -Rd[0] * 2.0;
        dQ[i+1] = -Rd[1] * 2.0;
    }
    hessian.resize(nx, nx);
    Eigen::MatrixXd QQ(nx, nx);
    for (int i=0; i<nx; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
    }
    for (int i=nx; i<nnzQ; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
        hessian.insert(jcolQ[i], irowQ[i]) = dQ[i];
    }

    // equality constraints
    MPCState temp = now_state;
    getLinearModel(temp, dref(1, 0));
    int my = dimx;
    double b[my];
    const int nnzA = 12 * T - 5;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];
    Eigen::Vector3d temp_vec(temp.x, temp.y, temp.theta);
    Eigen::Vector3d temp_b = A*temp_vec + C;
    
    for (int i=0; i<dimx; i++)
    {
        irowA[i] = jcolA[i] = i;
        dA[i] = 1;
    }
    b[0] = temp_b[0];
    b[1] = temp_b[1];
    b[2] = temp_b[2];
    irowA[dimx] = 0;
    jcolA[dimx] = dimx;
    dA[dimx] = -B(0, 0);
    irowA[dimx+1] = 1;
    jcolA[dimx+1] = dimx;
    dA[dimx+1] = -B(1, 0);
    irowA[dimx+2] = 2;
    jcolA[dimx+2] = dimx;
    dA[dimx+2] = -B(2, 0);
    irowA[dimx+3] = 2;
    jcolA[dimx+3] = dimx+1;
    dA[dimx+3] = -B(2, 1);
    int ABidx = 9*T - 9;
    int ABbegin = dimx+4;
    for (int i=0, j=1; i<ABidx; i+=9, j++)
    {
        getLinearModel(xbar[j], dref(1, j));
        for (int k=0; k<3; k++)
        {
            b[3*j+k] = C[k];
            irowA[ABbegin + i + k] = 3*j + k;
            jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 3;
            dA[ABbegin + i + k] = -A(k, k);
        }
        irowA[ABbegin + i + 3] = 3*j;
        jcolA[ABbegin + i + 3] = 3*j - 1;
        dA[ABbegin + i + 3] = -A(0, 2);

        irowA[ABbegin + i + 4] = 3*j + 1;
        jcolA[ABbegin + i + 4] = 3*j - 1;
        dA[ABbegin + i + 4] = -A(1, 2);
        
        irowA[ABbegin + i + 5] = 3*j;
        jcolA[ABbegin + i + 5] = dimx + 2*j;
        dA[ABbegin + i + 5] = -B(0, 0);
        
        irowA[ABbegin + i + 6] = 3*j + 1;
        jcolA[ABbegin + i + 6] = dimx + 2*j;
        dA[ABbegin + i + 6] = -B(1, 0);

        irowA[ABbegin + i + 7] = 3*j + 2;
        jcolA[ABbegin + i + 7] = dimx + 2*j;
        dA[ABbegin + i + 7] = -B(2, 0);
        
        irowA[ABbegin + i + 8] = 3*j + 2;
        jcolA[ABbegin + i + 8] = dimx + 2*j + 1;
        dA[ABbegin + i + 8] = -B(2, 1);
    }

    // iequality constraints
    const int mz  = 2 * T - 2;
    const int nnzC = 2 * dimu - 4;
    int   irowC[nnzC];
    int   jcolC[nnzC];
    double   dC[nnzC];
    for (int i=0, k=0; i<mz; i+=2, k+=4)
    {
        irowC[k] = i;
        jcolC[k] = dimx  + i;
        dC[k] = -1.0;

        irowC[k+1] = i;
        jcolC[k+1] = jcolC[k] +2;
        dC[k+1] = 1.0;

        irowC[k+2] = i + 1;
        jcolC[k+2] = dimx + 1 + i;
        dC[k+2] = -1.0;

        irowC[k+3] = i + 1;
        jcolC[k+3] = jcolC[k+2] +2;
        dC[k+3] = 1.0;
    }

    // xlimits and all
    int mx = dimu;
    int nc = mx+my+mz;
    lowerBound.resize(nc);
    upperBound.resize(nc);
    linearMatrix.resize(nc, nx);
    for (int i=0; i<mx; i+=2)
    {
        lowerBound[i] = min_speed;
        lowerBound[i+1] = -max_steer;
        upperBound[i] = max_speed;
        upperBound[i+1] = max_steer;
        linearMatrix.insert(i, dimx+i) = 1;
        linearMatrix.insert(i+1, dimx+i+1) = 1;
    }

    for (int i=0; i<nnzA; i++)
    {
        linearMatrix.insert(irowA[i]+mx, jcolA[i]) = dA[i];
    }

    for (int i=0; i<my; i++)
    {
        lowerBound[mx+i] = upperBound[mx+i] = b[i];
    }

    for (int i=0; i<nnzC; i++)
    {
        linearMatrix.insert(irowC[i]+mx+my, jcolC[i]) = dC[i];
    }

    for (int i=0; i<mz; i+=2)
    {
        lowerBound[mx+my+i] = -max_cv;
        upperBound[mx+my+i] = max_cv;
        lowerBound[mx+my+i+1] = -max_csteer;
        upperBound[mx+my+i+1] = max_csteer;
    }

    return;
    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);
    if(!solver.data()->setHessianMatrix(hessian)) return;
    if(!solver.data()->setGradient(gradient)) return;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return;
    if(!solver.data()->setLowerBound(lowerBound)) return;
    if(!solver.data()->setUpperBound(upperBound)) return;

    // instantiate the solver
    if(!solver.initSolver()) return;

    // controller input and QPSolution vector
    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;

    // get the controller input
    QPSolution = solver.getSolution();
    std::cout<<"Solution: v0=%f"<<QPSolution[dimx]<<"delta0=%f"<< QPSolution[dimx+1] <<std::endl;
    for (int i=0; i<dimu; i+=2)
    {
        output(0, i) = QPSolution[dimx+i];
        output(1, i) = QPSolution[dimx+i+1];
    }
}

void MPC::getCmd(void)
{
    int iter;

    double begin = Clock::NowInSeconds();
    for (iter=0; iter<max_iter; iter++)
    {
        predictMotion();
        last_output = output;
        if (control_a)
            solveMPCA();
        else
            solveMPCV();
        double du = 0;
        for (int i=0; i<output.cols(); i++)
        {
            du = du + fabs(output(0, i) - last_output(0, i))+ fabs(output(1, i) - last_output(1, i));
        }
        // break;
        if (du <= du_th || (Clock::NowInSeconds()-begin)>0.01)
        {
            break;
        }
    }
    if (iter == max_iter)
    {
        std::cout<<"MPC Iterative is max iter"<<std::endl;;
    }

    //predictMotion(xopt);
    //drawRefPath();
    //drawPredictPath(xopt);
    if (control_a)
    {
        //cmd.acceleration = min(max(output(0, 0), -max_accel),max_accel);
        // cmd.drive.speed = -0.4;
        //cmd.speed = now_state.v + dt * cmd.acceleration;
    }
    //else
        //cmd.speed = output(0, 0);
    //cmd.steering_angle = output(1, 0);
}
}
}