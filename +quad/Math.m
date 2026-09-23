classdef Math
    % Scalar-first Hamilton quaternions. Vectorized where stated.
    methods (Static)
        function S=skew(v)
            S=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
        end
        function q=normalize(q)
            % Algebraic norm preserves complex-step derivatives.
            q=q./sqrt(sum(q.^2,1));
        end
        function q=mul(a,b)
            q=[a(1,:).*b(1,:)-sum(a(2:4,:).*b(2:4,:),1); ...
               a(1,:).*b(2:4,:)+b(1,:).*a(2:4,:)+cross(a(2:4,:),b(2:4,:),1)];
        end
        function q=conj(q)
            q(2:4,:)=-q(2:4,:);
        end
        function R=rot(q)
            q=quad.Math.normalize(q); w=q(1); x=q(2); y=q(3); z=q(4);
            R=[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y); ...
               2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x); ...
               2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)];
        end
        function q=fromRot(R)
            tr=trace(R);
            if tr>0
                s=2*sqrt(tr+1); q=[s/4;(R(3,2)-R(2,3))/s;(R(1,3)-R(3,1))/s;(R(2,1)-R(1,2))/s];
            else
                [~,k]=max(diag(R));
                switch k
                    case 1
                        s=2*sqrt(1+R(1,1)-R(2,2)-R(3,3));
                        q=[(R(3,2)-R(2,3))/s;s/4;(R(1,2)+R(2,1))/s;(R(1,3)+R(3,1))/s];
                    case 2
                        s=2*sqrt(1+R(2,2)-R(1,1)-R(3,3));
                        q=[(R(1,3)-R(3,1))/s;(R(1,2)+R(2,1))/s;s/4;(R(2,3)+R(3,2))/s];
                    otherwise
                        s=2*sqrt(1+R(3,3)-R(1,1)-R(2,2));
                        q=[(R(2,1)-R(1,2))/s;(R(1,3)+R(3,1))/s;(R(2,3)+R(3,2))/s;s/4];
                end
            end
            q=quad.Math.normalize(q);
        end
        function q=exp(v)
            a=norm(v);
            if a<1e-8, q=quad.Math.normalize([1;v/2]);
            else, q=[cos(a/2);sin(a/2)*v/a]; end
        end
        function v=log(q)
            q=quad.Math.normalize(q); if q(1)<0, q=-q; end
            a=norm(q(2:4));
            if a<1e-10, v=2*q(2:4); else, v=2*atan2(a,q(1))*q(2:4)/a; end
        end
        function q=fromEuler(r,p,y)
            q=quad.Math.mul(quad.Math.mul([cos(y/2);0;0;sin(y/2)], ...
                [cos(p/2);0;sin(p/2);0]),[cos(r/2);sin(r/2);0;0]);
        end
        function a=euler(q)
            R=quad.Math.rot(q);
            a=[atan2(R(3,2),R(3,3));asin(max(-1,min(1,-R(3,1))));atan2(R(2,1),R(1,1))];
        end
        function a=wrap(a)
            a=atan2(sin(a),cos(a));
        end
    end
end
