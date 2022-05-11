import edu.wpi.first.wpilibj.controller.StateSpaceControllerCoeffs;
import edu.wpi.first.wpilibj.controller.StateSpaceLoop;
import edu.wpi.first.wpilibj.controller.StateSpaceObserverCoeffs;
import edu.wpi.first.wpilibj.controller.StateSpacePlantCoeffs;
import edu.wpi.first.wpiutil.math.*;
import edu.wpi.first.wpiutil.math.numbers.*;

public class DiffSwerveCoeffs {
  public static StateSpacePlantCoeffs<N3, N2, N3> makeDiffSwervePlantCoeffs() {
    Matrix<N3, N3> A = MatrixUtils.mat(Nat.N3(), Nat.N3()).fill(1.0, 0.003385150198164847, -0.0011398911039782148, 0.0, 0.35947684152175735, -0.40344534369888346, 0.0, 0.6724089061648054, 0.8974039664536022);
    Matrix<N3, N2> B = MatrixUtils.mat(Nat.N3(), Nat.N2()).fill(0.008201312741315397, 0.0024382157880213223, 3.0636650532009155, 0.7020176767823547, -1.1700294613039242, 2.1276414841577753);
    Matrix<N3, N3> C = MatrixUtils.mat(Nat.N3(), Nat.N3()).fill(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    Matrix<N3, N2> D = MatrixUtils.mat(Nat.N3(), Nat.N2()).fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    return new StateSpacePlantCoeffs<N3, N2, N3>(Nat.N3(), Nat.N2(), Nat.N3(), A, B, C, D);
  }

  public static StateSpaceControllerCoeffs<N3, N2, N3>
    makeDiffSwerveControllerCoeffs() {
    Matrix<N2, N3> K = MatrixUtils.mat(Nat.N2(), Nat.N3()).fill(4.109361474689985, 0.08499014906730384, -0.14062843057653684, -0.5936469626100538, 0.12948381291614233, 0.0912257662643821);
    Matrix<N2, N3> Kff = MatrixUtils.mat(Nat.N2(), Nat.N3()).fill(0.0007795019998080929, 0.2898766463158315, -0.09564601551033325, 0.0005383164581840219, 0.1594082337297932, 0.4174064550714919);
    Matrix<N2, N1> Umin = MatrixUtils.mat(Nat.N2(), Nat.N1()).fill(-1.0, -1.0);
    Matrix<N2, N1> Umax = MatrixUtils.mat(Nat.N2(), Nat.N1()).fill(1.0, 1.0);
    return new StateSpaceControllerCoeffs<N3, N2, N3>(K, Kff, Umin, Umax);
  }

  public static StateSpaceObserverCoeffs<N3, N2, N3>
    makeDiffSwerveObserverCoeffs() {
    Matrix<N3, N3> K = MatrixUtils.mat(Nat.N3(), Nat.N3()).fill(0.9901952585277586, 6.570320044406658e-09, 4.9091183297165255e-09, 6.570320044413538e-07, 0.9996002065918845, -1.9219432772579172e-08, 4.909118329717226e-07, -1.9219432772575473e-08, 0.9996003607875137);
    return new StateSpaceObserverCoeffs<N3, N2, N3>(K);
  }

  public static StateSpaceLoop<N3, N2, N3> makeDiffSwerveLoop() {
    return new StateSpaceLoop<N3, N2, N3>(makeDiffSwervePlantCoeffs(),
                                          makeDiffSwerveControllerCoeffs(),
                                          makeDiffSwerveObserverCoeffs());
  }
}
