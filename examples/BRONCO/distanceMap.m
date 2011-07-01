function distanceMap(cspace, maxPenetrationDistance, distancefilename, gridfilename)
%This function computes the distance map of a 3D grid (cspace) with voxels 
%with values 1 or 0 (1 means free space and 0 obstacle space), and stores 
%the result in a file named distancefilename. 
%Penetration depths are reported also up to a distance
%maxPenetrationDistance.
%Also a vrml file is saved with the VRML model of the 3D grid with colors
%as a function of the distances.
%
%distanceMap(Amira_imageReconstructedBin_mat,5,'broncoDistanceMap.txt','broncoGrid.wrl')
%
    s=size(cspace);%cells per axis
    dim=size(s,2);%dimension of the cspace
    index=1;
    %Scan all cells. Those that are obstacle cells and have a non-obstacle
    %neighbor cell is a border cell. Border cells are marked with the
    %distance matrix set to 0 and the origin matrix set to the number of
    %the cell.
    d=zeros(s);
    O=zeros(s);
    for k=1:1:s(3)%pisos
      k
      for j=1:1:s(2)%columnes
        for i=1:1:s(1)%files
          %Set the value of the distance matrix and of the origin matrix to
          %-2 indicating that they are not to be processed. This is then
          %changed for free cells and for border cells, i.e. only obstacle
          %cells that are not on the border will keep the -2 value.
          d(i,j,k)=-1000000;
          
          %Verify if it is an obstacle cell
          if cspace(i,j,k)==0 
              ok=0;
              if i-1>=1 & cspace(i-1,j,k)==1 
                  ok=1;
              elseif i+1<=s(1) & cspace(i+1,j,k)==1
                  ok=1;
              elseif j-1>=1 & cspace(i,j-1,k)==1
                  ok=1;
              elseif j+1<=s(2) & cspace(i,j+1,k)==1
                  ok=1;
              elseif k-1>=1 & cspace(i,j,k-1)==1
                  ok=1;
              elseif k+1<=s(3) & cspace(i,j,k+1)==1
                  ok=1;
              end
              
              %If a neighbor cell is a free cell then mark the cell as
              %border cell by loading the distance matrix and the origin
              %matrix, and by filling the first row of list matrix L that
              %contains the cells to be processed later.
              if ok==1
                d(i,j,k)=0;
                L(1,index) = s(2)*s(1)*(k-1) + s(1)*(j-1) + i;
                L2(1,index) = s(2)*s(1)*(k-1) + s(1)*(j-1) + i;
                index = index+1;
              end
          %If not an obstacle cell then the distance matrix and the origin
          %matrix are initialized to -1 indicating that thir value should
          %be set later.
          else 
            d(i,j,k)=1000000;
          end
        end
      end
    end
    
%     cspace
%     d
%     O

    %l contains the current list of cells to be processed. It is
    %initialized to the first row of matrix of lists L.
    lindex=1; %indicates the row of L being processed
    l=L(1,:)
    
    %Process all lists, until no more lists available.
    while lindex >= 1
        lindex
        lindex_i=0;
        %l %print current list being processed
        
        %SCAN all cells of the list l
        for n=1:1:length(l)
            %since lists are stored in a matrix L they all have the same
            %legth, but only the values different from 0 are to be
            %considered
            if l(n)==0 break 
            end
                   
            %compute the indices i,j,k of the label of the n-th cell in the list l: l(n)
            k = floor((l(n)-1)/(s(1)*s(2)))+1;%pis
            l2 = mod(l(n)-1,s(1)*s(2))+1;
            j = floor((l2-1)/s(1))+1;%columna
            i = mod(l2-1,s(1))+1;%fila
              
            %SCAN all neighbor cells
            inserted=0;
            if i-1>=1 
                %Verify if a neighbor cell belongs to the free cspace and has 
                %not yet been labelled with the distance (i.e. has matrix d
                %still at -1). If this is the case, label the distance by
                %increasing the current distance by 1. Then add the
                %neighbor cell to the next list to be processed in the next
                %pass of the algorithm (the next expansion of the wave).
               if d(i-1,j,k)==1000000 
                 d(i-1,j,k) = d(i,j,k)+1;
                 lindex_i = lindex_i + 1;
                 L(lindex+1,lindex_i) = s(2)*s(1)*(k-1) + s(1)*(j-1) + i - 1;
               end
            end
              
              if i+1<=s(1) 
                  if d(i+1,j,k)==1000000
                    d(i+1,j,k) = d(i,j,k)+1;
                    lindex_i = lindex_i + 1;
                     L(lindex+1,lindex_i) = s(2)*s(1)*(k-1) + s(1)*(j-1) + i + 1;
                  end
              end
              
              if j-1>=1 
                  if d(i,j-1,k)==1000000
                    d(i,j-1,k) = d(i,j,k)+1;
                    lindex_i = lindex_i + 1;
                    L(lindex+1,lindex_i) = s(2)*s(1)*(k-1) + s(1)*(j-2) + i;
                  end
              end
              
              if j+1<=s(2) 
                  if d(i,j+1,k)==1000000
                    d(i,j+1,k) = d(i,j,k)+1;
                    lindex_i = lindex_i + 1;
                    L(lindex+1,lindex_i) = s(2)*s(1)*(k-1) + s(1)*(j) + i;
                  end
              end
              
              
%               if k-1>=1 
%                   if d(i,j,k-1)==1000000
%                     d(i,j,k-1) = d(i,j,k)+1;
%                     lindex_i = lindex_i + 1;
%                     L(lindex+1,lindex_i) = s(2)*s(1)*(k-2) + s(1)*(j-1) + i;
%                   end
%               end
%               
%               if k+1<=s(3) 
%                   if d(i,j,k+1)==1000000
%                     d(i,j,k+1) = d(i,j,k)+1;
%                     lindex_i = lindex_i + 1;
%                     L(lindex+1,lindex_i) = s(2)*s(1)*(k) + s(1)*(j-1) + i;
%                   end
%               end
              
        end
        
               
        %Update List l
        %If at least a cell has been added to the new list then proceed 
        if lindex_i >0
            l = L(lindex+1,:);
            lindex = lindex+1;
            %L
        %otherwise the algorithm has reached its end
        else
            maxdistance = lindex %print the maximum level of lists proceesed
            lindex=-1;%set flag to terminate the while loop
        end
    end
    
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %REPEAT FOR NEGATIVE DISTANCES
     if maxPenetrationDistance >0
       lindex=1; %indicates the row of L being processed
       l=L2(1,:);
    
       %Process all lists, until no more lists available.
       while lindex >= 1
         %lindex
         lindex_i=0;
         %l %print current list being processed
        
         %SCAN all cells of the list l
         for n=1:1:length(l)
            %since lists are stored in a matrix L they all have the same
            %legth, but only the values different from 0 are to be
            %considered
            if l(n)==0 break 
            end
                   
            %compute the indices i,j,k of the label of the n-th cell in the list l: l(n)
            k = floor((l(n)-1)/(s(1)*s(2)))+1;%pis
            l2 = mod(l(n)-1,s(1)*s(2))+1;
            j = floor((l2-1)/s(1))+1;%columna
            i = mod(l2-1,s(1))+1;%fila
              
            %SCAN all neighbor cells
            inserted=0;
            if i-1>=1 
                %Verify if a neighbor cell belongs to the free cspace and has 
                %not yet been labelled with the distance (i.e. has matrix d
                %still at -1). If this is the case, label the distance by
                %increasing the current distance by 1. Then add the
                %neighbor cell to the next list to be processed in the next
                %pass of the algorithm (the next expansion of the wave).
               if d(i-1,j,k)==-1000000 
                 d(i-1,j,k) = d(i,j,k)-1;
                 lindex_i = lindex_i + 1;
                 L2(lindex+1,lindex_i) = s(2)*s(1)*(k-1) + s(1)*(j-1) + i - 1;
               end
            end
              
              if i+1<=s(1) 
                  if d(i+1,j,k)==-1000000
                    d(i+1,j,k) = d(i,j,k)-1;
                    lindex_i = lindex_i + 1;
                     L2(lindex+1,lindex_i) = s(2)*s(1)*(k-1) + s(1)*(j-1) + i + 1;
                  end
              end
              
              if j-1>=1 
                  if d(i,j-1,k)==-1000000
                    d(i,j-1,k) = d(i,j,k)-1;
                    lindex_i = lindex_i + 1;
                    L2(lindex+1,lindex_i) = s(2)*s(1)*(k-1) + s(1)*(j-2) + i;
                  end
              end
              
              if j+1<=s(2) 
                  if d(i,j+1,k)==-1000000
                    d(i,j+1,k) = d(i,j,k)-1;
                    lindex_i = lindex_i + 1;
                    L2(lindex+1,lindex_i) = s(2)*s(1)*(k-1) + s(1)*(j) + i;
                  end
              end
              
              
%               if k-1>=1 
%                   if d(i,j,k-1)==-1000000
%                     d(i,j,k-1) = d(i,j,k)-1;
%                     lindex_i = lindex_i + 1;
%                     L2(lindex+1,lindex_i) = s(2)*s(1)*(k-2) + s(1)*(j-1) + i;
%                   end
%               end
%               
%               if k+1<=s(3) 
%                   if d(i,j,k+1)==-1000000
%                     d(i,j,k+1) = d(i,j,k)-1;
%                     lindex_i = lindex_i + 1;
%                     L2(lindex+1,lindex_i) = s(2)*s(1)*(k) + s(1)*(j-1) + i;
%                   end
%               end
              
          end
        
               
          %Update List l
          %If at least a cell has been added to the new list then proceed 
          if lindex_i >0
            l = L2(lindex+1,:);
            lindex = lindex+1;
            %L
          %otherwise the algorithm has reached its end
          else
            lindex %print the maximum level of lists proceesed
            lindex=-1;%set flag to terminate the while loop
          end
        
          %stop at 25
          if lindex>=maxPenetrationDistance
             lindex=-1;
          end
       end
     end
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    
    %Print the distance-map
    fid = fopen(distancefilename, 'wt');
    fprintf(fid, '%d %d %d\n', [s(1) s(2) s(3)]);
    %origin of the grid
    ox=0%-146.953;%-10.258%
    oy=0%-142.031;%-2.536%
    oz=0%-315.75;%-237.565%
    fprintf(fid, '%d %d %d\n', ox, oy, oz);
    %size of the voxels
    sx=0.702807;
    sy=0.70277;
    sz=0.625492;
    fprintf(fid, '%d %d %d\n',sx, sy, sz);
    for k=1:1:s(3)%pisos
      for j=1:1:s(2)%columnes
        for i=1:1:s(1)%files
          if d(i,j,k)>-1000000 && d(i,j,k)<1000000 
            q = s(2)*s(1)*(k-1) + s(1)*(j-1) + i;
            fprintf(fid, '%d %d\n', [q d(i,j,k)]);
          end
        end
      end
    end
    fclose(fid)
    
    
    %Print the Grid in a VRML file
    gridStep = 1
    fidgrid = fopen(gridfilename, 'wt');
    fprintf(fidgrid, '#VRML V1.0 ascii\n');
    fprintf(fidgrid, 'DEF grid Separator {\n');
    fprintf(fidgrid, 'Separator {\n');
    fprintf(fidgrid, 'Material {\n');
    fprintf(fidgrid, 'diffuseColor [\n');
    for k=1:gridStep:s(3)%pisos
      for j=1:gridStep:s(2)%columnes
        for i=1:gridStep:s(1)%files
          if d(i,j,k)>-1000000 & d(i,j,k)<=0 
             if k==s(3) & (j==s(2) & i==s(1)) 
                 fprintf(fidgrid, '%f %f %f\n', 0.1,0.1,0.1);
             else
                fprintf(fidgrid, '%f %f %f,\n', 0.1,0.1,0.1);
             end
          elseif d(i,j,k)>0 & d(i,j,k)<1000000 
             c = 0.4+(d(i,j,k)/maxdistance);
             if(c>1) 
                 c=1;
             end
             if k==s(3) & j==s(2) & i==s(1) 
                 fprintf(fidgrid, '%f %f %f\n', c,c,c);
             else
                 fprintf(fidgrid, '%f %f %f,\n', c,c,c);
             end
          end
        end
      end
    end
    fprintf(fidgrid, ']\n');
    fprintf(fidgrid, '}\n');
    fprintf(fidgrid, 'MaterialBinding { value PER_PART  }\n'); %OVERALL
    fprintf(fidgrid, 'Coordinate3 {\n');
    fprintf(fidgrid, 'point [\n');
    for k=1:gridStep:s(3)%pisos
      for j=1:gridStep:s(2)%columnes
        for i=1:gridStep:s(1)%files
          if d(i,j,k)>-1000000 && d(i,j,k)<1000000 
            fprintf(fidgrid, '%f %f %f,\n', [ox+i*sx-sx/2 oy+j*sy-sy/2 oz+k*sz-sz/2]);
          end
        end
      end
    end
    %Add origin and top of bounding box
    fprintf(fidgrid, '%f %f %f,\n', [ox oy oz]);
    fprintf(fidgrid, '%f %f %f\n', [ox+s(1)*sx oy+s(2)*sy oz+s(3)*sz]);
    fprintf(fidgrid, ']\n');
    fprintf(fidgrid, '}\n');
    fprintf(fidgrid, 'DrawStyle { pointSize 3 }\n');
    fprintf(fidgrid, 'PointSet { }\n');
    fprintf(fidgrid, '}\n');
    fprintf(fidgrid, '}\n');
    fclose(fidgrid)
end
  