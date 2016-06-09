% Projects vector a on vector b.
function vp = ffb_projectVectorOnVector( b, a)
vp = (dot(a,b)/dot(b, b)) * b;
end