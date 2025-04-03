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


#if !defined(_LINK_H)
#define _LINK_H

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbLinear.h>
#include <kautham/problem/element.h>
#include <kautham/util/kthutil/kauthamdefs.h>
#include <kautham/mt/mt.h>
#include <string>
#include <Eigen/Geometry>
#include <memory> // For std::shared_ptr

using namespace mt;

namespace Kautham{

/** \addtogroup Problem
 *  @{
 */

  //!  This class is definition of Link part in cinematic chain robot.
	/*!	This class defines a Link part in cinematic chain robot like 
	*		industrial robots, it contains the model of solid link in inventor and
	*		it has the  information about the transformation between the parent's frame
	*		and its own frame.
	*
	*		It use a standar  Denavit - Hartember description for simples
	*		movements denoted for \f$ \alpha \f$ , a, \f$ \theta \f$ and d parameters.
	*		
    *		There are used three ways to define a transformation, the Standard D-H,
    *		the Modified D-H and the URDF one, and they one has the respective transformation matrix
	*		associated.  The transformation matrix used in Standar method is:
	*		\f$ ^{i-1}A_{i}=\left[
	*			\begin{array}{cccc}
	*					C\theta	& -C\alpha S\theta & S\alpha S\theta & aC\theta \\
	*					S\theta & C\alpha C\theta & -S\alpha C\theta & aS\theta \\
	*					0	& S\alpha & C\alpha & d	\\
	*					0 & 0 & 0 & 1
	*			\end{array}
	*		\right] \f$
	*		
    *		The transfomation matrix corresponding to the Modified method is:
	*		\f$ ^{i-1}A_{i}=\left[
	*			\begin{array}{cccc}
	*					C\theta	        & -S\theta        & 0          & a \\
	*					C\alpha S\theta & C\alpha C\theta & -S\alpha   & -dS\alpha \\
	*					S\alpha S\theta	& S\alpha C\theta & C\alpha    & dC\alpha	\\
	*					0 & 0 & 0 & 1
	*			\end{array}
	*		\right] \f$
    *       The transfomation matrix corresponding to the URDF method is:
    *		\f$ ^{i-1}A_{i}=\left[
    *			\begin{array}{cccc}
    *					R(axis,theta) [3x3] & d*axis [3,1] \\
    *					0 [1x3]             & 1
    *			\end{array}
    *		\right] \f$
	*/

  class Link {
  public:
      //!	Constructor.
      /*!	This constructor receive two Inventor files, visual and collision, and a global scale for the
	  *		associated link and put this link in the origin of absolute frame. 
      *		You can build a complete robot, if you adding progresively a each Link
	  *		from absolute coordinates frame to final effector frame.*/
      Link(string ivFile, string collision_ivFile, double scale,
           APPROACH Type, bool useBBOX = false);

      //!	Constructor.
      /*!	This constructor receive two models, visual and collision, and a global scale for the
      *		associated link and put this link in the origin of absolute frame.
      *		You can build a complete robot, if you adding progresively a each Link
      *		from absolute coordinates frame to final effector frame.*/
      Link(SoSeparator *visual_model, SoSeparator *collision_model, double scale,
           APPROACH Type, bool useBBOX = false);

	  //! Function to set \f$ \alpha \f$ parameter.
	  /*!	This function set Denavit - Hartemberg \f$ \alpha \f$ parameter.*/
    inline void         setAlpha(double alp){if(!armed)alpha = alp;}

	  //! Function to set \f$ a \f$ parameter.
	  /*!	This function set Denavit - Hartemberg \f$ a \f$ parameter.*/
    inline void         setA(double a){if(!armed)this->a = a;}
  	
	  //! Function to set \f$ \theta \f$ parameter.
	  /*!	This function set Denavit - Hartemberg \f$ \theta \f$ parameter.*/
    inline void         setTheta(double th){if(!armed)theta = th;}

	  //! Function to set \f$ d \f$ parameter.
	  /*!	This function set Denavit - Hartemberg \f$ d \f$ parameter.*/
    inline void         setD(double d){if(!armed)this->d = d;}

      //! Function to set \f$ axis \f$ parameter.
      /*!	This function sets \f$ axis \f$ parameter.*/
    inline void         setAxis(Unit3 axis){if(!armed) {this->axis = axis; this->axis_ << axis[0], axis[1], axis[2];}}
    inline void         setAxis(Eigen::Vector3d _axis){if(!armed)this->axis_ = _axis;}

	  //! Function to get \f$ d \f$ parameter.
	  /*!	This function get the current value from Denavit - Hartemberg \f$ d \f$ 
	  *		parameter.	*/
    inline double      getD() const {return d;}

	  //! Function to get \f$ \theta \f$ parameter.
	  /*!	This function get the current value from Denavit - Hartemberg \f$ \theta \f$ 
	  *		parameter.*/
    inline double      getTheta() const {return theta;}

	  //! Function to get \f$ a \f$ parameter.
	  /*!	This function get the current value from Denavit - Hartemberg \f$ a \f$ 
	  *		parameter.	*/
    inline double      getA() const {return a;}

	  //! Function to get \f$ \alpha \f$ parameter.
	  /*!	This function get the current value from Denavit - Hartemberg \f$ \alpha \f$ 
	  *		parameter.	*/
    inline double      getAlpha() const {return alpha;}

      //! Function to get \f$ axis \f$ parameter.
      /*!	This function gets the current value from \f$ axis \f$
      *		parameter.	*/
    inline Unit3        getAxis() const {return axis;}

    //! Function to set \f$ ode \f$ element.
    /*!	This function sets \f$ ode \f$ element.*/
    inline void         setOde(ode_element ode){ode = ode;if(!armed)this->element->ode = ode;}


    //! Function to get \f$ ode \f$ element.
    /*!	This function gets the current value from \f$ ode \f$
          *		element.	*/
    inline ode_element getOde() const {return this->element->ode;}


	  //! Function to set movable parameter.
	  /*!	This function set the current value from a movable parameter. 
	  */
    inline void         setMovable(bool m){if(!armed)movable = m;}

	  //! Function to get movable parameter.
	  /*!	This function get the current value from a movable parameter. */
    inline bool         getMovable() const {return movable;}

	  //!	This method set the true value to the Link if it's rotational, 
	  //! false otherwise.
    inline void         setRotational(bool r){if(!armed)rotational = r;}

	  //! This method return true if the Link is rotational, false otherwise.
    inline bool         getRotational() const {return rotational;}
  	
	  //!	This member function return the value of armed attribute.
	  /*!	This member function return de value of armed attribute.
	  *		\return armed */
    inline bool         isArmed() const {return armed;}

	  //!	This member function returns the current transformation.
	  /*!	This member function returns the current absolute link transformation 
	  *		associated with the Link.  Its is the current position and its current
	  *		orientation.*/
    inline mt::Transform* getTransformation(){return &absoluteTransform;}
    
    Eigen::AffineCompact3d getTransformationEigen();
    std::shared_ptr<Eigen::AffineCompact3d> getTransformationEigenPtr();

    Eigen::AffineCompact3d applyJointConfiguration(const double _theta);
    
	double             getdhMatrix(int i,int j){return dhMatrix[i][j];}
      	
    inline double      getValue(){return value;}
    bool                setValue(double q);
    bool                setParameter(double p);
    void                setArmed();
  	
	  //!	It member function used to calculate new position and orientation.
	  /*!	It function calculate new  position and orientation after change 
	  *		articular variable and it use the absolute transformation matrix 
      *		of prior Link and multiply it with D-H (or URDF) own matrix.*/
    void                calculatePnO();
  	
	  //!	This member function set the Link name.
    inline void         setName(string nam){name=nam;}

    inline              string getName() const {return name;}
  		
    void                setDHPars(double theta, double d, double a, double alpha);
  	
	  //!	This member function return the parent pointer to previous Link.
    inline Link*        getParent(){if(armed)return parent;else return NULL;}
    inline std::string  getParentName(){if(armed)return parent->getName();else return "";}

    void                setParent(Link* par);
    unsigned int        addChild(Link* child);
    inline unsigned int numChilds(){return (unsigned int)childs.size();}
    //!	This member function returns the specified child
    inline Link*        getChild(unsigned i){if (i < childs.size()) {return childs.at(i);} else {return NULL;}}
    bool                setPreTransform(double x, double y, double z,
                                        double wx, double wy, double wz,
                                        double angle);
    inline bool         setPreTransform(double posori[7]){
      return setPreTransform( posori[0], posori[1],posori[2], posori[3],
                              posori[4], posori[5], posori[6]);
    }
  	
    void                setLimits(double low, double hi);
    double*            getLimits(bool low=true);
    inline double      getWeight() const {return weight;}
    inline void         setWeight(double w){weight = w;}
    inline double      parameter2Value(double &param) const { return param*(hiLimit - lowLimit) + lowLimit;}
    inline double      value2Parameter(double &value) const { return (value - lowLimit)/(hiLimit - lowLimit);}
    inline double      getZeroOffset() const {return zeroOffset;}

    inline Element*     getElement(){return element;}
    inline bool         forceChange(Link* who){if(who == parent){ hasChanged = true; return true;}return false;}
    inline bool         changed(){return hasChanged;}
    SoSeparator *getModel(bool tran);
    SoSeparator *getCollisionModel(bool tran);
    SoSeparator *getModelFromColl();
  private:
    //! This is the pointer to the element assigned to the link. This is the link model
    Element*            element;

	  //!	Pointer to prior Link in chain sequence from the absolute frame to the final effector frame.
    Link*               parent;
  	
	  //!	Vector of pointers to every Link child. It's useful with TREE type robots.
    std::vector<Link*>       childs;
  	
	  //!	\f$ \alpha \f$ parameter for D-H description.
    double             alpha;
  	
	  //!	\f$ a \f$ parameter for D-H description.
    double             a;
  	
      //!	\f$ \theta \f$ parameter for D-H and URDF description.
    double             theta;
  	
      //!	\f$ d \f$ parameter for D-H and URDF description.
    double             d;

      //!	\f$ axis \f$ parameter for URDF description.
    Unit3               axis;
    Eigen::Vector3d     axis_;
  	
	  //! This variable is true for rotational Links.
    bool                rotational;
  	
	  //! This variable is true if it can move together.
    bool                movable;
  	
      //! This variable is true when all robot's part are armed and then the D-H (or URDF) fixed
	  //!	parameters are not posible to change.
    bool                armed;
  	
	  //! It is the low limit for movable Link. In rotational Links normaly is - \f$ \pi \f$
    double             lowLimit;
  	
	  //! It is the hi limit for moveable Link. In rotational Links normaly is \f$ \pi \f$
    double             hiLimit;
  	
      //! It is the weight to compute weighted distances between configurations, defaults to 1;
    double             weight;
  	
	  //! Value beetwen Low Limit and Hi Limit.
    double             value;
  	
	  //! Value beetwen 0 and 1 corresponding to a value between low and hi limit 
	  //!	and used to set the home position.
    double             zeroOffset;

	  //!	This matrix is the absolute transformation for the Link.
    mt::Transform       absoluteTransform;
  	
	  //!	Current Denavit - Hartemberg matrix.
    double             dhMatrix[4][4];

      //! This could be store an additional transformation between the link before and the dhMatrix
    mt::Transform*      preTransform;
    Eigen::AffineCompact3d origin_;
  	
	  //!	The name of Link.
    string              name;
  	
	  //! Approach used to describe this robot.
    APPROACH          Type;

    //! Used to define when the Link has been moved.
    bool                hasChanged;

    bool                changeChilds();

  };


  /** @}   end of Doxygen module "Problem" */
}
#endif  //_Link_H
