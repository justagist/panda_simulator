/***************************************************************************
* Methods from orocos_kdl and sns_ik library. Isolated to avoid 
* memory leak bugs. Removed dependency on sns_ik.

*
* @package: panda_gazebo
* @metapackage: panda_simulator
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

#include <panda_gazebo/kdl_methods.h>

namespace panda_gazebo
{

bool KDLMethods::initialise(KDL::Chain& chain)
// chain_(chain),nj(chain.getNrOfJoints()),ns(chain.getNrOfSegments()),
//         X(ns),S(ns),v(ns),a(ns),f(ns)
{
    chain_= chain;
    nj = chain_.getNrOfJoints();
    ns = chain_.getNrOfSegments() ;
    X = std::vector<KDL::Frame>(ns);
    S = std::vector<KDL::Twist>(ns);
    v = std::vector<KDL::Twist>(ns);
    a = std::vector<KDL::Twist>(ns);
    f = std::vector<KDL::Wrench>(ns);
    g_ag=-KDL::Twist(KDL::Vector(0.0, 0.0, -9.8),KDL::Vector::Zero());
    c_ag=-KDL::Twist(KDL::Vector::Zero(),KDL::Vector::Zero());

    locked_joints_ = std::vector<bool>(nj,false);

    nr = 0;
    jntarraynull = KDL::JntArray(nj);

    wrenchnull = std::vector<KDL::Wrench>(ns,KDL::Wrench::Zero());
    Ic = std::vector<KDL::ArticulatedBodyInertia, Eigen::aligned_allocator<KDL::ArticulatedBodyInertia> >(ns);

    return true;
}


int KDLMethods::PosFKJntToCart(const KDL::JntArray& q_in, KDL::Frame& p_out, int seg_nr)    {
    unsigned int segmentNr;
    if(seg_nr<0)
        segmentNr=chain_.getNrOfSegments();
    else
        segmentNr = seg_nr;

    p_out = KDL::Frame::Identity();
    if(q_in.rows()!=chain_.getNrOfJoints()){
        return -4;
    }
    else if(segmentNr>chain_.getNrOfSegments())
        return -3;
    else{
        int j=0;
        for(unsigned int i=0;i<segmentNr;i++){
            if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None){
                p_out = p_out*chain_.getSegment(i).pose(q_in(j));
                j++;
            }else{
                p_out = p_out*chain_.getSegment(i).pose(0.0);
            }
        }
        return 0;
    }
}

int KDLMethods::RNECartToJnt(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const KDL::Wrenches& f_ext,KDL::JntArray &torques, const KDL::Twist ag)
    {

        unsigned int j=0;

        
        //Sweep from root to leaf
        for(unsigned int i=0;i<ns;i++){
            double q_,qdot_,qdotdot_;
            if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None){
                q_=q(j);
                qdot_=q_dot(j);
                qdotdot_=q_dotdot(j);
                j++;
            }else
                q_=qdot_=qdotdot_=0.0;
            
          // ROS_WARN_NAMED("this","%d %f %f %f",i, q_, qdot_, qdotdot_);
            //Calculate segment properties: X,S,vj,cj
            X[i]=chain_.getSegment(i).pose(q_);//Remark this is the inverse of the 
                                                  //frame for transformations from 
                                                  //the parent to the current coord frame
            //Transform velocity and unit velocity to segment frame
            KDL::Twist vj=X[i].M.Inverse(chain_.getSegment(i).twist(q_,qdot_));
            S[i]=X[i].M.Inverse(chain_.getSegment(i).twist(q_,1.0));
            //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
            //calculate velocity and acceleration of the segment (in segment coordinates)
            if(i==0){
                v[i]=vj;
                a[i]=X[i].Inverse(ag)+S[i]*qdotdot_+v[i]*vj;
            }else{
                v[i]=X[i].Inverse(v[i-1])+vj;
                a[i]=X[i].Inverse(a[i-1])+S[i]*qdotdot_+v[i]*vj;
            }
            //Calculate the force for the joint
            //Collect RigidBodyInertia and external forces
            KDL::RigidBodyInertia Ii=chain_.getSegment(i).getInertia();
            // std::cout << i << " " << S[i].rot.x() << " " << S[i].rot.y() << " "   << std::endl;
            f[i]=Ii*a[i]+v[i]*(Ii*v[i])-f_ext[i];
            // std::cout << i << " " << f[i].force.x() << " " << f[i].force.y() << " "   << std::endl;
      //std::cout << "a[i]=" << a[i] << "\n f[i]=" << f[i] << "\n S[i]" << S[i] << std::endl;
        }
        //Sweep from leaf to root
        j=nj-1;
        for(int i=ns-1;i>=0;i--){
            if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None){
                torques(j)=dot(S[i],f[i]);
                torques(j)+=chain_.getSegment(i).getJoint().getInertia()*q_dotdot(j);  // add torque from joint inertia
                --j;
                // ROS_WARN_STREAM_NAMED("this","%d %f",i,torques(j));
            }
            if(i!=0)
                f[i-1]=f[i-1]+X[i]*f[i];
        }
  return 0;
    }

int KDLMethods::JacobianJntToJac(const KDL::JntArray& q_in, KDL::Jacobian& jac, int seg_nr)
    {
        if(nj != chain_.getNrOfJoints() || ns != chain_.getNrOfSegments())
            return -7;

        unsigned int segmentNr;
        if(seg_nr<0)
            segmentNr=chain_.getNrOfSegments();
        else
            segmentNr = seg_nr;

        //Initialize Jacobian to zero since only segmentNr columns are computed
        KDL::SetToZero(jac) ;

        if( q_in.rows()!=chain_.getNrOfJoints() || jac.columns() != chain_.getNrOfJoints())
            return -3;
        else if(segmentNr>chain_.getNrOfSegments())
            return -4;

        T_tmp = KDL::Frame::Identity();
        KDL::SetToZero(t_tmp);
        int j=0;
        int k=0;
        KDL::Frame total;
        for (unsigned int i=0;i<segmentNr;i++) {
            //Calculate new Frame_base_ee
            if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None){
                //pose of the new end-point expressed in the base
                total = T_tmp*chain_.getSegment(i).pose(q_in(j));
                //changing base of new segment's twist to base frame if it is not locked
                //t_tmp = T_tmp.M*chain_.getSegment(i).twist(1.0);
                if(!locked_joints_[j])
                    t_tmp = T_tmp.M*chain_.getSegment(i).twist(q_in(j),1.0);
            }else{
                total = T_tmp*chain_.getSegment(i).pose(0.0);

            }

            //Changing Refpoint of all columns to new ee
            KDL::changeRefPoint(jac,total.p-T_tmp.p,jac);

            //Only increase jointnr if the segment has a joint
            if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None){
                //Only put the twist inside if it is not locked
                if(!locked_joints_[j])
                    jac.setColumn(k++,t_tmp);
                j++;
            }

            T_tmp = total;
        }
        return 0;
    }

    //calculate inertia matrix H
    int KDLMethods::JntToMass(const KDL::JntArray &q, KDL::JntSpaceInertiaMatrix& H)
    {
        if(nj != chain_.getNrOfJoints() || ns != chain_.getNrOfSegments())
            return -7;
    //Check sizes when in debug mode
        if(q.rows()!=nj || H.rows()!=nj || H.columns()!=nj )
            return -3;
        unsigned int k=0;
    double q_;
    
    //Sweep from root to leaf
        for(unsigned int i=0;i<ns;i++)
    {
      //Collect RigidBodyInertia
          Ic[i]=chain_.getSegment(i).getInertia();
          if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None)
      {
          q_=q(k);
          k++;
      }
      else
      {
        q_=0.0;
      }
      X[i]=chain_.getSegment(i).pose(q_);//Remark this is the inverse of the frame for transformations from the parent to the current coord frame
      S[i]=X[i].M.Inverse(chain_.getSegment(i).twist(q_,1.0));  
        }
    //Sweep from leaf to root
        int j,l;
    k=nj-1; //reset k
        for(int i=ns-1;i>=0;i--)
    {
      
      if(i!=0)
        {
          //assumption that previous segment is parent
          Ic[i-1]=Ic[i-1]+X[i]*Ic[i];
        } 

      F=Ic[i]*S[i];
      if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None)
      {
          H(k,k)=dot(S[i],F);
          H(k,k)+=chain_.getSegment(i).getJoint().getInertia();  // add joint inertia
          j=k; //countervariable for the joints
          l=i; //countervariable for the segments
          while(l!=0) //go from leaf to root starting at i
        {
          //assumption that previous segment is parent
          F=X[l]*F; //calculate the unit force (cfr S) for every segment: F[l-1]=X[l]*F[l]
          l--; //go down a segment
          
          if(chain_.getSegment(l).getJoint().getType()!=KDL::Joint::None) //if the joint connected to segment is not a fixed joint
          {    
            j--;
            H(k,j)=dot(F,S[l]); //here you actually match a certain not fixed joint with a segment 
            H(j,k)=H(k,j);
          }
        } 
          k--; //this if-loop should be repeated nj times (k=nj-1 to k=0)
      }

    }
    return 0;
    }

    //calculate coriolis matrix C
    int KDLMethods::JntToCoriolis(const KDL::JntArray &q, const KDL::JntArray &q_dot, KDL::JntArray &coriolis)
    {
    //make a null matrix with the size of q_dotdot and a null wrench
    KDL::SetToZero(jntarraynull);

    
    //the calculation of coriolis matrix C
    return RNECartToJnt(q, q_dot, jntarraynull, wrenchnull, coriolis, c_ag);
    
    }

    //calculate gravity matrix G
    int KDLMethods::JntToGravity(const KDL::JntArray &q,KDL::JntArray &gravity)
    {

    //make a null matrix with the size of q_dotdot and a null wrench
    
    KDL::SetToZero(jntarraynull);
    //the calculation of coriolis matrix C
    return RNECartToJnt(q, jntarraynull, jntarraynull, wrenchnull, gravity, g_ag);
    }

}

