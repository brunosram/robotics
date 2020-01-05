clc
clear all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rect_from=cell(0);
theta_rect_f=0;
rect_to=cell(0);
theta_rect_t=0;
circ_from=cell(0);
theta_circ_f=0;
circ_to=cell(0);
theta_circ_t=0;
tri_from=cell(0);
theta_tri_f=0;
tri_to=cell(0);
theta_tri_t=0;
square_from=cell(0);
theta_sq_f=0;
square_to=cell(0);
theta_sq_t=0;
%%%%%%% BASE_ORIGIN  %%%%%%%%%%%%
ORIGIN_C=0;
ORIGIN_R=0;
%%%%%%%  BASE_LEFTDOWN  %%%%%%%%%%%%
 base_leftdown_c=0;
 base_leftdown_r=0;
 %%%%%% BASE_UPLEFT  %%%%%%%%%%
  base_upleft_r=0;
   base_upleft_c=0;
   %%%%% BASE_LEFTUP  %%%%%%%%%%%
   base_leftup_c=0;
   base_leftup_r=0;
   %%%%% BASE_DOWNLEFT %%%%%%%%
   base_downleft_r=0;
   base_downleft_c=0;
   %%%%%  BASE_LENGTH  %%%%%%%%%%%%
base_length=6;
scale=0;
%%%%%% BASE_TO_REF  %%%%%%%%%%%%%
x1=40.1;
y1=20.2;
  Q_refbase=[0,-1,0,x1;-1,0,0,y1;0,0,-1,0;0,0,0,1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[A]=imread('Picture 7.jpg');
A=flipud(A);
A=fliplr(A);
I=rgb2gray(A);
I=I(:,35:end-400);

% A=imread('pic4.jpg');
% I=rgb2gray(A);
% I=I(:,1:640);
% [A]=imread('pic4.jpg');
% A=flipud(A);
% A=fliplr(A);
% I=rgb2gray(A);
% I=I(:,1:end-400);




Bin=I;
Bin(I>100)=0;
Bin(I<=100)=1;
% figure
% imshow(Bin,[])
%CC=bwconncomp(Bin,4)
[L,n]=bwlabel(Bin);
% figure
% imshow(L,[])
for i=1:n
    [r, c]=find(L==i);
 if length(r)<=30
     L(find(L==i))=0;
 end
end

Bin=L;
Bin(Bin==0)=0;
Bin(Bin>0)=1;
% figure
% imshow(Bin,[])
[L,n]=bwlabel(Bin);
figure
imshow(L,[])
%%%%%%%%%%%%%%%%%%
icell=cell(1,n);
center=zeros(n,2);
%center_sub=zeros(n,2);
index=cell(1,n);
[r_L,c_L]=size(L);
extension=5;
%%%%%%%%%%%%%%%%%%%

   
%hold on
figuresize=zeros(1,n);
for i=1:n
    [r, c]=find(L==i);
 figuresize(i)=length(r);
end
base_ind=find(figuresize==max(figuresize));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:n
    if i==base_ind%%%BASE_REFERENCE

            [r, c]=find(L==i);
     icell{i}=L(max(1,min(r)-extension):min(max(r)+extension,r_L),max(1,min(c)-extension):min(max(c)+extension,r_L));
   index{i}=[r,c];
   mean_pix=floor(mean(index{i}));
   center(i,:)=mean_pix;
   
   bw=edge(icell{i},'sobel'); 
[sub_edge,num] = bwlabel(bw);
              figure
   imshow(sub_edge,[])
   hold on
          [r_sub, c_sub]=find(sub_edge==1);
    sub_figuresize(i)=length(r_sub);
   %mean_pix_sub=floor(mean([r_sub , c_sub]));
   %center_sub(i,:)=mean_pix;
  % plot(mean_pix_sub(2),mean_pix_sub(1),'r*')
   hold on
   %%%%%%%%  LEFT_UP %%%%%%%%%%%
   leftup_ind=find(c==min(c));
   base_leftup_c=min(c);
  base_leftup_r=min(r(leftup_ind));
 
   %%%%%%%%%%%%   UP_LEFT  %%%%%%
   upleft_ind=find(r==min(r));
   base_upleft_r=min(r);
   base_upleft_c=min(c(upleft_ind));
   
   %%%%%%%%%%%%   LEFT_DOWN  %%%%%%%%
      leftdown_ind=find(c==min(c));
   base_leftdown_c=min(c);
   base_leftdown_r=max(r(leftdown_ind));
  
        %%%%%%%%%%%%   DOWN_MINUS2_LEFT   %%%%%%%%
      downleft_ind=find(r==max(r)-2);
   base_downleft_r=max(r)-2;
   base_downleft_c=min(c(downleft_ind));
   %%%%%%%%%%%  FIND_ORIGIN  %%%%%%%%%
   %%%%%% ROTATE_INVERSE_CLOCKWISE %%%%%%
if (base_leftup_c<base_downleft_c&&base_downleft_c<=base_upleft_c&&(abs(base_leftdown_c-base_downleft_c)>8))||(base_downleft_c>=base_upleft_c&&base_downleft_c>=base_leftup_c)
        ORIGIN_C=base_downleft_c;
ORIGIN_R=base_downleft_r;
side_length=distance(ORIGIN_C,ORIGIN_R,base_leftup_c,base_leftup_r);
scale=base_length/side_length;

    else if abs(base_leftdown_c-base_downleft_c)<=8%% TO_BE_DISCUSSED
       ORIGIN_C=base_downleft_c;
ORIGIN_R=base_downleft_r;
side_length=abs(base_downleft_r-base_upleft_r);
scale=base_length/side_length;
else if  base_leftdown_c<=base_leftup_c&&base_leftdown_c<=base_upleft_c
  %%%%%% ROTATE_CLOCKWISE  %%%%%%%%%%
  ORIGIN_C=base_leftdown_c;
  ORIGIN_R=base_leftdown_r;
  side_length=distance(ORIGIN_C,ORIGIN_R,base_upleft_c,base_upleft_r);
scale=base_length/side_length;
        else
        pop_box = msgbox('Ambiguous Base');
        end
    end
end
      %%%%%%%%%  END_OF_FOR_ORIGIN  %%%%%%%
    
    else

    [r, c]=find(L==i);
     icell{i}=L(max(1,min(r)-extension):min(max(r)+extension,r_L),max(1,min(c)-extension):min(max(c)+extension,r_L));
   index{i}=[r,c];
   mean_pix=floor(mean(index{i}));
   center(i,:)=mean_pix;
   
   bw=edge(icell{i},'sobel'); 
[sub_edge,num] = bwlabel(bw);

   if num>1%THE DESTINATION IS A RING
       sub_edge(sub_edge==2)=0;
%           figure
%    imshow(sub_edge,[])
   end
      figure
   imshow(sub_edge,[])
      hold on
   %%%%%%%%%%%%%  PICKING_PEAk_POINTS   %%%%%%%%
  [r_sub, c_sub]=find(sub_edge==1);
    sub_figuresize(i)=length(r_sub);
   mean_pix_sub=floor(mean([r_sub , c_sub]));
   %center_sub(i,:)=mean_pix;
   plot(mean_pix_sub(2),mean_pix_sub(1),'r*')
   hold on
   %%%%%%%%  RIGHT_DOWN  %%%%%%%%%%%
   rightdown_ind=find(c_sub==max(c_sub));
   rightdown_c=max(c_sub);
   rightdown_r=max(r_sub(rightdown_ind));
   plot(rightdown_c,rightdown_r,'g*')
   hold on
   %%%%%%%%%%%%   UP_RIGHT  %%%%%%
   upright_ind=find(r_sub==min(r_sub));
   upright_r=min(r_sub);
   upright_c=max(c_sub(upright_ind));
   plot(upright_c,upright_r,'g*')
   hold on
   %%%%%%%%%%%%   LEFT_UP  %%%%%%%%
      leftup_ind=find(c_sub==min(c_sub));
   leftup_c=min(c_sub);
   leftup_r=min(r_sub(leftup_ind));
   plot(leftup_c,leftup_r,'g*')
   hold on
   %%%%%%%    CALCULATING_DISTANCE     %%%%%%%%
   rd_ur=distance(rightdown_c,rightdown_r,upright_c,upright_r)
   ur_lu=distance(upright_c,upright_r,leftup_c,leftup_r)
   %%     JUDGE_IF_RECTANGLE     %%%%%%%%%%%%
   if min(rd_ur,ur_lu)>=30&&abs(rd_ur-ur_lu)>=30
       if num>1
       text(mean_pix_sub(2),mean_pix_sub(1)  ...
      ,'Rectangular_destination','FontSize',14,'Color','r')
        rect_to(end+1)=cell(1);
        rect_to{end}=[mean_pix(1) mean_pix(2)];
        % ANGLE %
        %%%%%%%
        %%%%%%%%
          if rd_ur<ur_lu
          mid_c=0.5*(upright_c+leftup_c);
          mid_r=0.5*(upright_r+leftup_r);
         
          theta_rect_t=rad2deg(atan2(mid_c-mean_pix_sub(2) ,mean_pix_sub(1)-mid_r ));
          
          plot([mid_c,mean_pix_sub(2)],[mid_r,mean_pix_sub(1)],'r')
          else
          mid_c=0.5*(upright_c+rightdown_c);
          mid_r=0.5*(upright_r+rightdown_r);
          theta_rect_t=rad2deg(atan2(mid_c-mean_pix_sub(2),mean_pix_sub(1)-mid_r ));
          
          plot([mid_c,mean_pix_sub(2)],[mid_r,mean_pix_sub(1)],'r')
          end
          %%%%%%%%  END_ANGLE  %%%%%%%%%%%%%%
       else %%%  RECTANGULAR_FROM  %%%%%%%%%
             text(mean_pix_sub(2),mean_pix_sub(1)  ...
             ,'Rectangular_from','FontSize',14,'Color','r')
            rect_from(end+1)=cell(1);
            rect_from{end}=[mean_pix(1) mean_pix(2)];
    % ANGLE %
        %%%%%%%
        %%%%%%%%
          if rd_ur<ur_lu
                        mid_c=0.5*(upright_c+leftup_c);
          mid_r=0.5*(upright_r+leftup_r);
          theta_rect_f=rad2deg(atan2(mid_c-mean_pix_sub(2) ,mean_pix_sub(1)-mid_r ));
          
          plot([mid_c,mean_pix_sub(2)],[mid_r,mean_pix_sub(1)],'r')
          else
              mid_c=0.5*(upright_c+rightdown_c);
          mid_r=0.5*(upright_r+rightdown_r);

          theta_rect_f=rad2deg(atan2(mid_c-mean_pix_sub(2),mean_pix_sub(1)-mid_r ));
          
          plot([mid_c,mean_pix_sub(2)],[mid_r,mean_pix_sub(1)],'r')
          end
   %%%%%%%%%%%%%%  END_ANGLE  %%%%%%%%%%%
       end
   else%%%  NOT RECTANGLE  %%%%%%%%%
       rd_lu=distance(rightdown_c,rightdown_r,leftup_c,leftup_r)

       if (max([abs(rd_lu-rd_ur),abs(rd_lu-ur_lu),abs(ur_lu-rd_ur)])- ...
           min([abs(rd_lu-rd_ur),abs(rd_lu-ur_lu),abs(ur_lu-rd_ur)]) ...
       <=10)||(min([rd_lu,rd_ur,ur_lu])<=20)%% TRIANGLAR_TO  %%%
            if num>1
       text(mean_pix_sub(2),mean_pix_sub(1)  ...
       ,'tri_destination','FontSize',14,'Color','r')
  tri_to(end+1)=cell(1);
   tri_to{end}=[mean_pix(1) mean_pix(2)];
   %%%%%%%%%%%%TRIANGULAR_TO_ANGLE%%%%%%%%%%%%%%%%
   if upright_r ==leftup_r
     theta_tri_t=rad2deg(atan2(leftup_c-mean_pix_sub(2), ...
             mean_pix_sub(1)-leftup_r ));
             plot([leftup_c,mean_pix_sub(2)],[leftup_r,mean_pix_sub(1)],'r')
   else
   theta_tri_t=rad2deg(atan2(upright_c-mean_pix_sub(2), ...
    mean_pix_sub(1)-upright_r ));
   plot([upright_c,mean_pix_sub(2)],[upright_r,mean_pix_sub(1)],'r')          
   end
            else 
%% TRIANGLAR_FROM  %%%
                text(mean_pix_sub(2),mean_pix_sub(1)  ...
       ,'tri_from','FontSize',14,'Color','r')
   tri_from(end+1)=cell(1);
   tri_from{end}=[mean_pix(1) mean_pix(2)];
              %%%%%%%%%TRIANGUlAR_FROM_ANGLE%%%%%%%%
             if upright_r ==leftup_r
                  theta_tri_f=rad2deg(atan2(leftup_c-mean_pix_sub(2), ...
             mean_pix_sub(1)-leftup_r ));
             plot([leftup_c,mean_pix_sub(2)],[leftup_r,mean_pix_sub(1)],'r')
             else
             theta_tri_f=rad2deg(atan2(upright_c-mean_pix_sub(2), ...
             mean_pix_sub(1)-upright_r ));
             plot([upright_c,mean_pix_sub(2)],[upright_r,mean_pix_sub(1)],'r')
             end
       end
       else  %%% CIRCLE AND SQUARE  %%%%%%
         esti_size=rd_ur*ur_lu;

          if abs(esti_size-figuresize(i))<=1168&&num==1%%%SQUARE_FROM
           text(mean_pix_sub(2),mean_pix_sub(1)  ...
             ,'Square_from','FontSize',14,'Color','r')
             square_from(end+1)=cell(1)
             square_from{end}=[mean_pix(1) mean_pix(2)]
             %%%%%%%%  SQUARE_FROM_ANGLE%%%%%%%%%%
  mid_c=0.5*(upright_c+rightdown_c);
          mid_r=0.5*(upright_r+rightdown_r);
          theta_sq_f=rad2deg(atan2(mid_c-mean_pix_sub(2) ,mean_pix_sub(1)-mid_r ));
          
          plot([mid_c,mean_pix_sub(2)],[mid_r,mean_pix_sub(1)],'r')
             
             
          else%% SQUARE_TO  CIRCLE_TO  CIRCLE_FROM
              column_mid=floor(0.5*(mean_pix_sub(2)+rightdown_c))
              ind_mid=find(c_sub==column_mid)
              row_mid=min(r_sub(ind_mid))
              %mean_pix_sub(2),mean_pix_sub(1)
              plot(column_mid,row_mid,'y*')
              d=distance(column_mid,row_mid, ...
              mean_pix_sub(2),mean_pix_sub(1))
          r=distance(rightdown_c,rightdown_r, ...
              mean_pix_sub(2),mean_pix_sub(1))
          if abs(d-r)<=5%CIRCLE
                      if num==1  %%% CIRCLE_FROM
                         text(mean_pix_sub(2),mean_pix_sub(1)  ...
                    ,'Circ_destination','FontSize',14,'Color','r')
                    circ_from(end+1)=cell(1)
                     circ_from{end}=[mean_pix(1) mean_pix(2)]
                            
                     else if num==2  %% CIRCLE_TO
                            text(mean_pix_sub(2),mean_pix_sub(1)  ...
                         ,'circ_destination','FontSize',14,'Color','r')
                         circ_to(end+1)=cell(1)
                            circ_to{end}=[mean_pix(1) mean_pix(2)]
                        
                         end
                      end
          else %%% SQUARE_TO %%%%%%%%%%%
                    text(mean_pix_sub(2),mean_pix_sub(1)  ...
             ,'Square_destination','FontSize',14,'Color','r')
             square_to(end+1)=cell(1)
             square_to{end}=[mean_pix(1) mean_pix(2)]   
             %%%%%%%%%%%%%% SQUARE_TO_ANGLE%%%%%%%%%%
               mid_c=0.5*(upright_c+rightdown_c);
          mid_r=0.5*(upright_r+rightdown_r);
          theta_sq_t=rad2deg(atan2(mid_c-mean_pix_sub(2) ,mean_pix_sub(1)-mid_r ));
          
          plot([mid_c,mean_pix_sub(2)],[mid_r,mean_pix_sub(1)],'r')
             
       end
   end

       end

   end

       
       
    end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   figure
imshow(I)
hold on
   for i=1:length(rect_from)
         plot(rect_from{i}(2),rect_from{i}(1),'w*')
         hold on
        text(rect_from{i}(2),rect_from{i}(1)  ...
       ,'Rectangular_from','FontSize',14,'Color','r')
   end
   for i=1:length(rect_to)
         plot(rect_to{i}(2),rect_to{i}(1),'b*')
         hold on
        text(rect_to{i}(2),rect_to{i}(1)  ...
       ,'Rectangular_to','FontSize',14,'Color','r')
   end
   
   for i=1:length(tri_from)
         plot(tri_from{i}(2),tri_from{i}(1),'w*')
         hold on
        text(tri_from{i}(2),tri_from{i}(1)  ...
       ,'tri_from','FontSize',14,'Color','r')
   end
   for i=1:length(tri_to)
         plot(tri_to{i}(2),tri_to{i}(1),'b*')
         hold on
        text(tri_to{i}(2),tri_to{i}(1)  ...
       ,'tri_to','FontSize',14,'Color','r')
   end
   
   for i=1:length(square_from)
         plot(square_from{i}(2),square_from{i}(1),'w*')
         hold on
        text(square_from{i}(2),square_from{i}(1)  ...
       ,'square_from','FontSize',14,'Color','r')
   end
      for i=1:length(square_to)
         plot(square_to{i}(2),square_to{i}(1),'b*')
         hold on
        text(square_to{i}(2),square_to{i}(1)  ...
       ,'square_to','FontSize',14,'Color','r')
      end
   
         for i=1:length(circ_from)
         plot(circ_from{i}(2),circ_from{i}(1),'w*')
         hold on
        text(circ_from{i}(2),circ_from{i}(1)  ...
       ,'circ_from','FontSize',14,'Color','r')
   end
   for i=1:length(circ_to)
         plot(circ_to{i}(2),circ_to{i}(1),'b*')
         hold on
        text(circ_to{i}(2),circ_to{i}(1)  ...
       ,'circ_to','FontSize',14,'Color','r')
   end
   
   plot(ORIGIN_C,ORIGIN_R,'b*')
   text(ORIGIN_C,ORIGIN_R  ...
                         ,'ORIGIN','FontSize',14,'Color','r')
                     
 plot(base_leftup_c,base_leftup_r,'g*')
 plot(base_upleft_c,base_upleft_r,'g*')
  plot(base_downleft_c,base_downleft_r,'g*')   
  %%%%%%%  LOCATE_CENTIMETERS  %%%%%%%%%%%
  diff_c_sqf=-(ORIGIN_C-square_from{1}(2))*scale;
  diff_r_sqf=(ORIGIN_R-square_from{1}(1))*scale;
  
  diff_c_sqt=-(ORIGIN_C-square_to{1}(2))*scale;
  diff_r_sqt=(ORIGIN_R-square_to{1}(1))*scale;
  
  diff_c_cirf=-(ORIGIN_C-circ_from{1}(2))*scale;
  diff_r_cirf=(ORIGIN_R-circ_from{1}(1))*scale;
  
    diff_c_cirt=-(ORIGIN_C-circ_to{1}(2))*scale;
  diff_r_cirt=(ORIGIN_R-circ_to{1}(1))*scale;
  
  diff_c_recf=-(ORIGIN_C-rect_from{1}(2))*scale;
  diff_r_recf=(ORIGIN_R-rect_from{1}(1))*scale;
  
    diff_c_rect=-(ORIGIN_C-rect_to{1}(2))*scale;
  diff_r_rect=(ORIGIN_R-rect_to{1}(1))*scale;
  
  diff_c_trif=-(ORIGIN_C-tri_from{1}(2))*scale;
    diff_r_trif=(ORIGIN_R-tri_from{1}(1))*scale;
    
  diff_c_trit=-(ORIGIN_C-tri_to{1}(2))*scale;
    diff_r_trit  =(ORIGIN_R-tri_to{1}(2))*scale;
  %%%%%%%%%%%%Q_MATRICES%%%%%%%%%%%%%%%%%
  theta_tri_f= theta_tri_f+60;
  theta_tri_t=theta_tri_t+60;
  
%   
% Q_rectf=Q_refbase*[[cosd(theta_rect_f),-sind(theta_rect_f),0,diff_r_recf];
%                 [sind(theta_rect_f),cosd(theta_rect_f),0,diff_c_recf];
%                 [0,0,1,-1];
%                 [0,0,0,1]];
% Q_rectt=Q_refbase*[[cosd(theta_rect_t),-sind(theta_rect_t),0,diff_r_rect];
%                 [sind(theta_rect_t),cosd(theta_rect_t),0,diff_c_rect];
%                 [0,0,1,-1];
%                 [0,0,0,1]];
% Q_circt=Q_refbase*[[cosd(theta_circ_t),-sind(theta_circ_t),0,diff_r_cirt];
%                 [sind(theta_circ_t),cosd(theta_circ_t),0,diff_c_cirt];
%                 [0,0,1,-1];
%                 [0,0,0,1]];
% Q_circf=Q_refbase*[[cosd(theta_circ_f),-sind(theta_circ_f),0,diff_r_cirf];
%                 [sind(theta_circ_f),cosd(theta_circ_f),0,diff_c_cirf];
%                 [0,0,1,-1];
%                 [0,0,0,1]];
% Q_sqf=Q_refbase*[[cosd(theta_sq_f),-sind(theta_sq_f),0,diff_r_sqf];
%                 [sind(theta_sq_f),cosd(theta_sq_f),0,diff_c_sqf];
%                 [0,0,1,-1];
%                 [0,0,0,1]];            
% Q_sqt=Q_refbase*[[cosd(theta_sq_t),-sind(theta_sq_t),0,diff_r_sqt];
%                 [sind(theta_sq_t),cosd(theta_sq_t),0,diff_c_sqt];
%                 [0,0,1,-1];
%                 [0,0,0,1]];            
% Q_trif=Q_refbase*[[cosd(theta_tri_f),-sind(theta_tri_f),0,diff_r_trif];
%                 [sind(theta_tri_f),cosd(theta_tri_f),0,diff_c_trif];
%                 [0,0,1,-1];
%                 [0,0,0,1]];     
% Q_trit=Q_refbase*[[cosd(theta_tri_t),-sind(theta_tri_t),0,diff_r_trit];
%                 [sind(theta_tri_t),cosd(theta_tri_t),0,diff_c_trit];
%                 [0,0,1,-1];
%                 [0,0,0,1]];     
   %%%%%%%%%%% Psi should smaller than 0 %%%%%%%%%%%%%%%%%%  
   psi_rt=-30;%psi_rectangle_to
   psi_rf=-60;%psi_rectangle_from
   psi_tt=-90;
   psi_tf=-60;
   psi_st=-60;
   psi_sf=-60;
   psi_ct=-60;
   psi_cf=-60;
   
      Q_rectf=Q_refbase*[[cosd(theta_rect_f),-sind(theta_rect_f),0,diff_r_recf];
                [sind(theta_rect_f),cosd(theta_rect_f),0,diff_c_recf];
                [0,0,1,-1];
                [0,0,0,1]]*[1,0,0,0;0,cosd(psi_rf),-sind(psi_rf),0;0,sind(psi_rf),cosd(psi_rf),0;0,0,0,1];

            Q_rectt=Q_refbase*[[cosd(theta_rect_t),-sind(theta_rect_t),0,diff_r_rect];
                [sind(theta_rect_t),cosd(theta_rect_t),0,diff_c_rect];
                [0,0,1,-1];
                [0,0,0,1]]*[1,0,0,0;0,cosd(psi_rt),-sind(psi_rt),0;0,sind(psi_rt),cosd(psi_rt),0;0,0,0,1];

            Q_circt=Q_refbase*[[cosd(theta_circ_t),-sind(theta_circ_t),0,diff_r_cirt];
                [sind(theta_circ_t),cosd(theta_circ_t),0,diff_c_cirt];
                [0,0,1,-1];
                [0,0,0,1]]*[1,0,0,0;0,cosd(psi_ct),-sind(psi_ct),0;0,sind(psi_ct),cosd(psi_ct),0;0,0,0,1];

            Q_circf=Q_refbase*[[cosd(theta_circ_f),-sind(theta_circ_f),0,diff_r_cirf];
                [sind(theta_circ_f),cosd(theta_circ_f),0,diff_c_cirf];
                [0,0,1,-1];
                [0,0,0,1]]*[1,0,0,0;0,cosd(psi_cf),-sind(psi_cf),0;0,sind(psi_cf),cosd(psi_cf),0;0,0,0,1];

            Q_sqf=Q_refbase*[[cosd(theta_sq_f),-sind(theta_sq_f),0,diff_r_sqf];
                [sind(theta_sq_f),cosd(theta_sq_f),0,diff_c_sqf];
                [0,0,1,-1];
                [0,0,0,1]]*[1,0,0,0;0,cosd(psi_sf),-sind(psi_sf),0;0,sind(psi_sf),cosd(psi_sf),0;0,0,0,1];            

            Q_sqt=Q_refbase*[[cosd(theta_sq_t),-sind(theta_sq_t),0,diff_r_sqt];
                [sind(theta_sq_t),cosd(theta_sq_t),0,diff_c_sqt];
                [0,0,1,-1];
                [0,0,0,1]]*[1,0,0,0;0,cosd(psi_st),-sind(psi_st),0;0,sind(psi_st),cosd(psi_st),0;0,0,0,1];            

            Q_trif=Q_refbase*[[cosd(theta_tri_f),-sind(theta_tri_f),0,diff_r_trif];
                [sind(theta_tri_f),cosd(theta_tri_f),0,diff_c_trif];
                [0,0,1,-1];
                [0,0,0,1]]*[1,0,0,0;0,cosd(psi_tf),-sind(psi_tf),0;0,sind(psi_tf),cosd(psi_tf),0;0,0,0,1];     

            Q_trit=Q_refbase*[[cosd(theta_tri_t),-sind(theta_tri_t),0,diff_r_trit];
                [sind(theta_tri_t),cosd(theta_tri_t),0,diff_c_trit];
                [0,0,1,-1];
                [0,0,0,1]]*[1,0,0,0;0,cosd(psi_tt),-sind(psi_tt),0;0,sind(psi_tt),cosd(psi_tt),0;0,0,0,1];           
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%END_OF_PROGRAM%%%%%%%%%%%%%
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
%max(index{2}(:,2))- min(index{2}(:,2))
% figure
% imshow(bw,[])

% figure
% imshow(L_edge,[])
% for i=1:n
%     %%%%%%%%%%%%%%
%    size_r = size(r);
%     distance = zeros(size_r);
%     for j = 1:1:size_r(1)
%             distance(j) = sqrt((r(j)-mean_pix(1))^2 + (c(j)-mean_pix(2))^2);
%     end
% % %%%%%%%%%%%%%%
% x=1:size_r(1);
% x=x';
% p=polyfit(x,distance,7);
% f1=polyval(p,x);
% figure
% plot(x,f1)
% end
%%%%%%%%%%% 
%    %%%%%%%%%% BEGIN_TO_RECOGNITION  %%%%
%    [r_sub,c_sub]=find(sub_edge==1);
%     %%%%%%%%%%%%%%
%    size_r = size(r_sub);
%     distance = zeros(size_r);
%     for j = 1:1:size_r(1)
%             distance(j) = sqrt((r_sub(j)-mean_pix(1))^2 + (c_sub(j)-mean_pix(2))^2);
%     end
% % %%%%%%%%%%%%%%
% x=1:size_r(1);
% x=x';
% p=polyfit(x,distance,7);
% f1=polyval(p,x);
% figure
% plot(x,f1)


  % text(min(c),max(r),txt,'FontSize',14,'Color','r')
   
% hold on
% if distance(max(r_sub),
%    
% end
%%%%%%%%%%  DRAW_POINTS  %%%%%%%%%%
% if i==1
%     plot(min(c),max(r),'r*')
%     txt = 'base';
% text(min(c),max(r),txt,'FontSize',14,'Color','r')
% else
%     plot(mean_pix(1,2),mean_pix(1,1),'r*')
% end

%%%%%%%%%%%%%
%     figure
%     imshow( icell{1,i},[])
