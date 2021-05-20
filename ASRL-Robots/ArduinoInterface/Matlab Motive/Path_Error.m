% to compute error, we calculate the actual x,z coordinates of a circle, normalize
% the caputure postition and calcutlate the error: actual - caputured. to do so,
% some period for the circle must be given, some time period of the capture must be given.

function ERR = Path_Error(time,x_cap,y_cap,x_start,y_start)
    %assuming 8pi period
    for i = 1:10
    x_calc(i) = xpath_8pi(time(i),x_start);
    y_calc(i) = ypath_8pi(time(i),y_start);
    end
    for i = 1:10
    x_err(i) = (x_calc(i)- x_cap(i));
    y_err(i) = (y_calc(i)- y_cap(i));    
    end
    x_err = x_err';
    y_err = y_err';
    
    ERR(:,1) = x_err;
    ERR(:,2) = y_err;
end



