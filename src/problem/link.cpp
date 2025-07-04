/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */


#include <kautham/problem/link.h>

#if defined(KAUTHAM_USE_FCL)
#include <kautham/problem/ivfclelement.h>
#else
#include <kautham/problem/ivelement.h>
#endif

#include <kautham/mt/point3.h>


namespace Kautham {


  //!	Constructor.
  /*!	This constructor provide the way to create any Link and you progressively can build
  *		any cinematic chain robot from the absolute reference frame
  *		to Final Efector frame.
  *		\param ivFile is the path string in the Inventor file that contain the visual model from the link solid.
  *     \param collision_ivFile is the path string in the Inventor file that contain the collision model from the link solid.
  *		\param scale is the global scale for this link and It is only used for 
  *		graphical representation.
  */
  Link::Link(string ivFile, string collision_ivFile, double scale,
             APPROACH Type, bool useBBOX){
#if defined(KAUTHAM_USE_FCL)
      element = new IVFCLElement(ivFile,collision_ivFile,scale,useBBOX);
#endif
	  a = (double)0.0;
	  alpha = (double)0.0;
	  theta = (double)0.0;
	  d = (double)0.0;
	  movable = true;
	  rotational = true;
	  armed = false;
	  lowLimit = (double)-M_PI;
	  hiLimit = (double)M_PI;
	  value = (double)0.0;
      zeroOffset = (double)0.0;
	  parent = NULL;
	  childs.clear();
      this->Type = Type;
	  
      for(int i = 0; i < 4; ++i) {
          for(int j=0 ; j<4; ++j) {
              if (i == j) {
                  dhMatrix[i][j]= (double)1.0;
              } else {
                  dhMatrix[i][j]= (double)0.0;
              }
          }
      }

      hasChanged = false;
      preTransform = NULL;
  }


  //!	Constructor.
  /*!	This constructor provide the way to create any Link and you progressively can build
  *		any cinematic chain robot from the absolute reference frame
  *		to Final Efector frame.
  *		\param visual_model is the visual model from the link solid.
  *     \param collision_model is the collision model from the link solid.
  *     \param scale is the global scale for this link and It is only used for
  *		graphical representation.
  */
  Link::Link(SoSeparator *visual_model, SoSeparator *collision_model, double scale, APPROACH Type, bool useBBOX){
#if defined(KAUTHAM_USE_FCL)
      element = new IVFCLElement(visual_model,collision_model,scale,useBBOX);
#endif

      a = (double)0.0;
      alpha = (double)0.0;
      theta = (double)0.0;
      d = (double)0.0;
      movable = true;
      rotational = true;
      armed = false;
      lowLimit = (double)-M_PI;
      hiLimit = (double)M_PI;
      value = (double)0.0;
      zeroOffset = (double)0.0;
      parent = NULL;
      childs.clear();
      this->Type = Type;

      for(int i = 0; i < 4; ++i) {
          for(int j=0 ; j<4; ++j) {
              if (i == j) {
                  dhMatrix[i][j]= (double)1.0;
              } else {
                  dhMatrix[i][j]= (double)0.0;
              }
          }
      }

      hasChanged = false;
      preTransform = NULL;
  }


  //!	This member function set the value of armed attribute.
  /*!	This member function set the value of armed attribute, Its means that all 
  *		robot is armed, and its Links and its links are complete. When this 
  *		attribute is set to true is not posible to change fixed D - H parameters.*/
  void Link::setArmed(){
      if(!armed){
          armed = true;
          switch (Type) {
          case DHSTANDARD:
              dhMatrix[0][0] = cos(theta);							dhMatrix[0][1] = -cos(alpha)*sin(theta);
              dhMatrix[0][2] = sin(alpha)*sin(theta);	  dhMatrix[0][3] = a*cos(theta);

              dhMatrix[1][0] = sin(theta);							dhMatrix[1][1] = cos(alpha)*cos(theta);
              dhMatrix[1][2] = -sin(alpha)*cos(theta);	dhMatrix[1][3] = a*sin(theta);

              /*dhMatrix[2][0] = 0 */                   dhMatrix[2][1] = sin(alpha);
              dhMatrix[2][2] = cos(alpha);							dhMatrix[2][3] = d;
              break;
          case DHMODIFIED:
              dhMatrix[0][0] = cos(theta);							dhMatrix[0][1] = -sin(theta);
              /*dhMatrix[0][2] = 0;*/                   dhMatrix[0][3] = a;

              dhMatrix[1][0] = cos(alpha)*sin(theta);	  dhMatrix[1][1] = cos(alpha)*cos(theta);
              dhMatrix[1][2] = -sin(alpha);	            dhMatrix[1][3] = -d*sin(alpha);

              dhMatrix[2][0] = sin(alpha)*sin(theta);   dhMatrix[2][1] = sin(alpha)*cos(theta);
              dhMatrix[2][2] = cos(alpha);							dhMatrix[2][3] = d*cos(alpha);
              break;
          case URDF:
              //dhMatrix has default value: identity matrix
              break;
          }
      }
      hasChanged = true;
      calculatePnO();
  }


  //!	This function set a Link of variable value.
  /*! This funcition set a Link of variable value between 0 and 1, corresponding 
  *		to lowLimit and hiLimit.This function changes the value of the variable which
  *		depends directly on the configuration of the Linkt. This Function can be  
  *		rotarcional or telescopic and  for this reason it can changes the 
  *		value of \f$\theta\f$ o de \f$d\f$.
  *
  *		When the articular variable value has changed, we have to
  *		calculate  the transformation again with regard to absolute origin	
  *		that the associated Link  has had, that done it based on the
  *		transformation of the  previous link (prior) and the D-H parameters.
  *		This is possible only if the Link is armed and movable.
  */
  bool Link::setValue(double q){
      if(armed && movable){
          if(q <= lowLimit)
              q = lowLimit;
          if(q >= hiLimit)
              q= hiLimit;

          value = q;
          q += zeroOffset;
          if(rotational){
              if(theta != q)
                  hasChanged = true;
              theta = q;
          }else{
              if(d != q)
                  hasChanged = true;
              d = q	;
          }

          calculatePnO();
          return true;
      }
      return false;
  }


  bool Link::setParameter(double p){
    if(armed && movable){
      if(p < (double)0.0) p=0;
	  else if(p > (double) 1.0) p=1;
      return setValue( parameter2Value(p) );

      //if(p >= (double)0.0 && p<= (double) 1.0)
      //  return setValue( parameter2Value(p) );
      return false;
    }
    return false;
  }

  void Link::calculatePnO(){
	  //SbVec3f transTemp,  scaleTemp;
	  //SbRotation rotTemp, scaoriTemp;
	  double tranv[3], rotv[4];

    if(hasChanged){ 
      switch (Type) {
      case DHSTANDARD:
	      if(rotational){
              dhMatrix[0][0] = cos(theta);							dhMatrix[0][1] = -cos(alpha)*sin(theta);
		      dhMatrix[0][2] = sin(alpha)*sin(theta);	  dhMatrix[0][3] = a*cos(theta);
		      dhMatrix[1][0] = sin(theta);							dhMatrix[1][1] = cos(alpha)*cos(theta);
		      dhMatrix[1][2] = -sin(alpha)*cos(theta);	dhMatrix[1][3] = a*sin(theta);
	      }else{
		      dhMatrix[2][3] = d;
	      }
          break;
      case DHMODIFIED:
	      if(rotational){
		      dhMatrix[0][0] = cos(theta);							dhMatrix[0][1] = -sin(theta);
		      dhMatrix[1][0] = cos(alpha)*sin(theta);	  dhMatrix[1][1] = cos(alpha)*cos(theta);
		      dhMatrix[2][0] = sin(alpha)*sin(theta);   dhMatrix[2][1] = sin(alpha)*cos(theta);
	      }else{
		      dhMatrix[1][3] = -d*sin(alpha);           dhMatrix[2][3] = d*cos(alpha);
	      }
          break;
      case URDF:
          if(rotational){//only rotattion has changed
              Rotation tmp_rotation(axis,theta);
              Matrix3x3 tmp_matrix = tmp_rotation.getMatrix();
              for(int i = 0;i < 3;i++) {
                  for(int j = 0;j < 3;j++){
                      dhMatrix[i][j] = tmp_matrix[i][j];
                  }
              }
          }else{//only traslation has changed
              for(int i = 0;i < 3;i++){
                  dhMatrix[i][3] = d*axis[i];
              };
          }
          break;
      }
    
      mt::Rotation tempRot(Matrix3x3(dhMatrix[0][0],dhMatrix[0][1],dhMatrix[0][2],
                           dhMatrix[1][0],dhMatrix[1][1],dhMatrix[1][2],
                           dhMatrix[2][0],dhMatrix[2][1],dhMatrix[2][2]));
      mt::Point3 tempTran(dhMatrix[0][3],dhMatrix[1][3],dhMatrix[2][3]);
  	  
      mt::Transform dhTemp(tempRot, tempTran);
  									 
	    if(parent != NULL){
		    absoluteTransform = *(parent->getTransformation());
      }

      if(preTransform != NULL)
        absoluteTransform *= *preTransform;

      absoluteTransform *= dhTemp;

      tempTran = absoluteTransform.getTranslation();
      tempRot = absoluteTransform.getRotation();
      for(int j=0; j<3; j++){
        tranv[j] = tempTran[j];
        rotv[j] = tempRot[j];
      }
      rotv[3] = tempRot[3];
     
      element->setPosition(tranv);
      element->setOrientation(rotv);

      for(unsigned int i = 0; i < childs.size(); i++){
        ((Link*)childs[i])->forceChange(this);
        ((Link*)childs[i])->calculatePnO();
      }
      
      hasChanged = false;
    } 
  }

  Eigen::AffineCompact3d Link::getTransformationEigen() {
    mt::Rotation rot = absoluteTransform.getRotation();
    mt::Matrix3x3 rotMatrix = rot.getMatrix();
    mt::Point3 pos = absoluteTransform.getTranslation();

    // Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z);
    Eigen::AffineCompact3d transformation = 
        Eigen::Translation3d(pos[0], pos[1], pos[2]) * 
        Eigen::Quaterniond(rot[3], rot[0], rot[1], rot[2]);

    return transformation;
  }

  std::shared_ptr<Eigen::AffineCompact3d> Link::getTransformationEigenPtr() {
    mt::Rotation rot = absoluteTransform.getRotation();
    mt::Matrix3x3 rotMatrix = rot.getMatrix();
    mt::Point3 pos = absoluteTransform.getTranslation();

    // Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z);
    return std::make_shared<Eigen::AffineCompact3d>(
        Eigen::Translation3d(pos[0], pos[1], pos[2]) * 
        Eigen::Quaterniond(rot[3], rot[0], rot[1], rot[2])
    );
  }

    Eigen::AffineCompact3d Link::applyJointConfiguration(const double _theta) {

        Eigen::AffineCompact3d theta_rotation = Eigen::AffineCompact3d::Identity();

        // Ensure the axis is normalized
        Eigen::Vector3d u = this->axis_.normalized();

        // Compute trigonometric terms
        double cos_theta = std::cos(_theta);
        double sin_theta = std::sin(_theta);
        double one_minus_cos = 1.0 - cos_theta;

        // Compute cross-product matrix [u]_x
        Eigen::Matrix3d cross_product_matrix;
        cross_product_matrix <<  0,      -u.z(),   u.y(),
                                u.z(),   0,     -u.x(),
                                -u.y(),   u.x(),   0;

        // Compute outer product (u ⊗ u)
        Eigen::Matrix3d outer_product = u * u.transpose();

        // Compute rotation matrix R using Rodrigues' formula:
        theta_rotation.linear() = cos_theta * Eigen::Matrix3d::Identity()
                                + sin_theta * cross_product_matrix
                                + one_minus_cos * outer_product;


        theta_rotation = this->origin_ * theta_rotation;
        
        return theta_rotation;
    }


  //!	This function set all D-H parameters. Angles are in degrees.  
  //! The theta parameter of D-H is the zeroOffset to home position.
  void Link::setDHPars(double theta, double d, double a, double alpha){
	  this->theta = theta;
	  this->d = d;
	  this->a = a;
	  this->alpha = alpha;
	  if(rotational)
		  zeroOffset = this->theta;
	  else
		  zeroOffset = this->d;
  }

  //! This member function set low and hi limits to articular variable. 
  //! If limits represents angles, they are in radians. Otherwise there are in meters.
  void Link::setLimits(double low, double hi){
	  if(!armed){
		  this->lowLimit = low;
		  this->hiLimit = hi;
	  }
  }

  double* Link::getLimits(bool low){
    if(low)
      return &lowLimit;
    else
      return &hiLimit;
  }

  void Link::setParent(Link* par){
    if(!armed){
      parent=par;
      par->addChild(this);
      setArmed();
    }
  }

  unsigned int Link::addChild(Link* child){
    childs.push_back(child);
    return childs.size();
  }

  bool Link::changeChilds(){
    if(childs.size()>0){
      for(unsigned int i = 0; i < childs.size(); i++)
        ((Link*)childs[i])->forceChange(this);
      return true;
    }
    return false;
  }

  bool Link::setPreTransform(double x, double y, double z,
                             double wx, double wy, double wz, double angle){
      if (Type != URDF) {
          if(preTransform == NULL){
              mt::Point3 tempTran(x,y,z);
              mt::Rotation tempRot(mt::Unit3(wx,wy,wz),angle);
              preTransform = new mt::Transform(tempRot,tempTran);
              return true;
          } else
              return false;
      } else {
          mt::Rotation tempRot(wz,wy,wx);//rotatation around fixed axis: first roll around x axis,
          //then pitch around y axis and finally yaw around z axis
          mt::Point3 tempTran(x,y,z);//translation
          preTransform = new mt::Transform(tempRot,tempTran);

        this->origin_ = Eigen::AffineCompact3d::Identity();
        this->origin_.translation() = Eigen::Vector3d(x,y,z);
        /*
        URDF uses extrinsic rotations (fixed-axis rotations) by default.
        This means the rotations are applied relative to the static world axes (X, Y, Z), not relative to the current rotated axes.
        */
        this->origin_.linear() =
            Eigen::AngleAxisd(wx, Eigen::Vector3d::UnitX()).toRotationMatrix() * // Roll (X-axis)
            Eigen::AngleAxisd(wy, Eigen::Vector3d::UnitY()).toRotationMatrix() * // Pitch (Y-axis)
            Eigen::AngleAxisd(wz, Eigen::Vector3d::UnitZ()).toRotationMatrix();   // Yaw (Z-axis)
        
        return true;
      }
  }


  SoSeparator *Link::getModel(bool tran) {
      if (element != NULL) {
          return (((IVElement*)element)->ivModel(tran));
      } else {
          SoSeparator *sep = new SoSeparator;
          return sep;
      }
  }


  SoSeparator *Link::getCollisionModel(bool tran)  {
      if (element != NULL) {
          return (((IVElement*)element)->collision_ivModel(tran));
      } else {
          SoSeparator *sep = new SoSeparator;
          return sep;
      }
  }


  SoSeparator *Link::getModelFromColl() {
    if (element != NULL) {
    #if defined(KAUTHAM_USE_FCL)
          return (((IVFCLElement*)element)->getIvFromFCLModel());
    #endif
    }else {
          SoSeparator *sep = new SoSeparator;
          return sep;
      }
    }
}
