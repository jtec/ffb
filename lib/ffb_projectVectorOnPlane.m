% Projects vector b on a plane defined by a vector n that is normal to the
% plane
function vp = ffb_projectVectorOnPlane( a, n)
vp = a - ffb_projectVectorOnVector(a, n);
end