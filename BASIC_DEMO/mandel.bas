'MANDEL

Z$=".,'~=+:;*%&$OXB#@ "
Time$="00:00:00"

F=50
For Y = -12 To 12
   For X = -49 To 29

      C=X*229/100
      D=Y*416/100
      A=C:B=D:I=0
d200:
      Q=B/F:S=B-(Q*F)
      T=((A*A)-(B*B))/F+C
      B=2*((A*Q)+(A*S/F))+D
      A=T: P=A/F:Q=B/F
      If ((P*P)+(Q*Q))>=5 GoTo d280
      I=I+1:If I<16 GoTo d200
      Print " ";
      GoTo d290
d280:
      Print Mid$(Z$,I+1,1);
d290:

   Next X
   Print ""

Next Y
Print "Finished", Time$                                   