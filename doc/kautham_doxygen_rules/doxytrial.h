/**
* \mainpage THIS IS MY PROJECT
*
* This is my project description. It is created to select the Doxygen rules to follow.
*
* Documentation of mathematica expression is done using LaTeX syntaxis
* written between \\f$ anf \\f$, e.g.:
\f$
^{i-1}A_{i}=\left[
   \begin{array}{cccc}
          C\theta         & -S\theta        & 0        & a \\
      C\alpha S\theta & C\alpha C\theta & -S\alpha & -dS\alpha \\
          S\alpha S\theta & S\alpha C\theta & C\alpha  & dC\alpha \\
          0               & 0               & 0        & 1
   \end{array}
\right]
\f$
 *
 *
 *
 *  \todo This details what is still missing in the documentation!!!
 *  The list of todo tasks can be disabled with the doxywizard in order to generate a clean (but unfinished) documentation.
 *
*/


//! Namespace MyNamespace contains the my classes
namespace MyNamespace {
/**
 *  @defgroup label_group1 The Name of Group 1
 *  \brief Brief description: This is a brief description of Group 1, it ends with a dot.
 *
 *  Detailed description: This is a detailed description of Group 1
 *  @{
 */


//! This is a brief description of class C1 in group 1, written in the header file
class C1 {
    public:
    int i;//!< Description of data member function is written in the header file
        C1();//!< This is a brief description of the constructor written in the header file
        //! Brief descritpion: this brief description is writen in the header file
        void function1_C1();
        inline void function2_C1();//!< Brief description: This is an inline function. This info is written in the header file
    private:
    int j; //!< Description of data member function is written in the header file
};
/** @} */ // end of group1
////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////
/** \addtogroup label_group1
 *  @{
 */
/** @brief This is a brief description of class C2 in group 1, written in the header file */
class C2 {
public:
    int i;//!< Description of data member function is written in the header file
     C2();//!< This is a brief description of the constructor written in the header file
     //! Brief descritpion: this brief description is writen in the header file
     int function1_C2(int p);
     inline void function2_C2();//!< Brief description: This is an inline function. This info is written in the header file
};
/** @} */ // end of group1
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////

/**
 *  @defgroup label_group2 The Name of Group 2
 *  \brief Brief description: This is a brief description of Group 2, it ends with a dot.
 *
 *  Detailed description: This is a detailed description of Group 2
 *  @{
 */


//! This is a brief description of class C3 in group 2, written in the header file
class C3 {
public:
    int i;//!< Description of data member function is written in the header file
    C3();//!< This is a brief description of the constructor written in the header file
    //! Brief descritpion: this brief description is writen in the header file
     void function1_C3();
     inline void function2_C3();//!< Brief description: This is an inline function. This info is written in the header file
};
/** @} */ // end of group2
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
/** \addtogroup label_group2
 *  @{
 */

//!  This is a brief description of class C4 in group 3, written in the header file
class C4{
public:
    int i;
    C4();//!< This is a brief description of the constructor written in the header file
    //! Brief descritpion: this brief description is writen in the header file
    int function1_C2(int p);
    inline void function2_C4();//!< Brief description: This is an inline function. This info is written in the header file
};
/** @} */ // end of group2
////////////////////////////////////////////////////////////


}


