#ifndef KKL_G2O_EDGE_SE3_PRIORVEC_HPP
#define KKL_G2O_EDGE_SE3_PRIORVEC_HPP

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
  // IMU测量的加速度的边
class EdgeSE3PriorVec : public g2o::BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, g2o::VertexSE3> {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeSE3PriorVec()
		: g2o::BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, g2o::VertexSE3>()
		{}
               
		void computeError() override {
		       // 获取该边的节点
			const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
          
			Eigen::Vector3d direction = _measurement.head<3>();     // 参考方向      设定为Z轴方向
			Eigen::Vector3d measurement = _measurement.tail<3>();    // 测量方向
                        // 获取节点状态    v1->estimate().linear()获取节点姿态      R*g_measurement = g  =>  R^-1 * g = g_measurement       这里即获取重力的测量值   
			Eigen::Vector3d estimate = (v1->estimate().linear().inverse() * direction);
                        // 残差           重力测量值 - 真实测量值          这里误差的构建也太粗糙了    应为IMU加速度的测量值还与自身运动有关啊   这里只考虑重力的影响了
			_error = estimate - measurement;
		}
                  
		void setMeasurement(const Eigen::Matrix<double, 6, 1>& m) override {
		        // 加速度向量归一化      
			_measurement.head<3>() = m.head<3>().normalized();
			_measurement.tail<3>() = m.tail<3>().normalized();
		}

		virtual bool read(std::istream& is) override {
			Eigen::Matrix<double, 6, 1> v;
			is >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5];
    		setMeasurement(v);
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j) {
					is >> information()(i, j);
					if (i != j)
						information()(j, i) = information()(i, j);
				}
			return true;
		}
		virtual bool write(std::ostream& os) const override {
			Eigen::Matrix<double, 6, 1> v = _measurement;
			os << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " " << v[4] << " " << v[5];
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j)
					os << " " << information()(i, j);
			return os.good();
		}
	};
}

#endif
