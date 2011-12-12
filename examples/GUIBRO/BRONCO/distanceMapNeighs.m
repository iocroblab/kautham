function distanceMapNeighs(cspace, filename,filenameNeighs)
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
      for j=1:1:s(2)%columnes
        for i=1:1:s(1)%files
          %Set the value of the distance matrix and of the origin matrix to
          %-2 indicating that they are not to be processed. This is then
          %changed for free cells and for border cells, i.e. only obstacle
          %cells that are not on the border will keep the -2 value.
          d(i,j,k)=-2;
          
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
                index = index+1;
              end
          %If not an obstacle cell then the distance matrix and the origin
          %matrix are initialized to -1 indicating that thir value should
          %be set later.
          else 
            d(i,j,k)=-1;
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
    l=L(1,:);
        
    sindex=1; %index of cells in the skeleton
          
    %START searching for the skeleton by growing the border of cells as an
    %expaning wave:
    
    initialized = 0;
    count=0;
    %Process all lists, until no more lists available.
    while lindex >= 1
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
              
            %find neighs if cell is a free cell (i.e. d to be determined or
            %positive.)
            if d(i,j,k) == -1  | d(i,j,k)> 0
              count = count+1;
              if initialized == 0 
                neigh = [l(n) -1 -1 -1 -1 -1 -1];
                initialized = 1;
              else
                neigh = [neigh ; [l(n) -1 -1 -1 -1 -1 -1]];
              end
            end
            %SCAN all neighbor cells
            inserted=0;
            neighcount=1;
            if i-1>=1 
                %Verify if a neighbor cell belongs to the free cspace and has 
                %not yet been labelled with the distance (i.e. has matrix d
                %still at -1). If this is the case, label the distance by
                %increasing the current distance by 1. Then add the
                %neighbor cell to the next list to be processed in the next
                %pass of the algorithm (the next expansion of the wave).
               if d(i-1,j,k)==-1 
                 d(i-1,j,k) = d(i,j,k)+1;
                 lindex_i = lindex_i + 1;
                 neighlabel= s(2)*s(1)*(k-1) + s(1)*(j-1) + i - 1;
                 L(lindex+1,lindex_i) = neighlabel;     
                 if d(i,j,k) == -1  | d(i,j,k)> 0
                   neighcount = neighcount+1;
                   neigh(count,neighcount)= neighlabel;
                 end
               elseif d(i-1,j,k)>0 
                 if d(i,j,k) == -1 | d(i,j,k)> 0
                   neighlabel= s(2)*s(1)*(k-1) + s(1)*(j-1) + i - 1;
                   neighcount = neighcount+1;
                   neigh(count,neighcount)= neighlabel;
                 end
               end
            end
              
              if i+1<=s(1) 
                  if d(i+1,j,k)==-1
                    d(i+1,j,k) = d(i,j,k)+1;
                    lindex_i = lindex_i + 1;
                    neighlabel= s(2)*s(1)*(k-1) + s(1)*(j-1) + i + 1;
                     L(lindex+1,lindex_i) = neighlabel;
                     if d(i,j,k) == -1 | d(i,j,k)> 0
                        neighcount = neighcount+1;
                        neigh(count,neighcount)= neighlabel;
                    end
                  elseif d(i+1,j,k)>0
                     if d(i,j,k) == -1 | d(i,j,k)> 0
                        neighlabel= s(2)*s(1)*(k-1) + s(1)*(j-1) + i + 1;
                        neighcount = neighcount+1;
                         neigh(count,neighcount)= neighlabel;
                     end
                  end
              end
              
              if j-1>=1 
                  if d(i,j-1,k)==-1
                    d(i,j-1,k) = d(i,j,k)+1;
                    lindex_i = lindex_i + 1;
                    neighlabel=  s(2)*s(1)*(k-1) + s(1)*(j-2) + i;
                    L(lindex+1,lindex_i) =neighlabel;
                     if d(i,j,k) == -1 | d(i,j,k)> 0
                        neighcount = neighcount+1;
                        neigh(count,neighcount)= neighlabel;
                     end
                  elseif d(i,j-1,k)>0
                     if d(i,j,k) == -1 | d(i,j,k)> 0
                        neighlabel= s(2)*s(1)*(k-1) + s(1)*(j-2) + i;
                        neighcount = neighcount+1;
                        neigh(count,neighcount)= neighlabel;
                     end
                  end
              end
              
              if j+1<=s(2) 
                  if d(i,j+1,k)==-1
                    d(i,j+1,k) = d(i,j,k)+1;
                    lindex_i = lindex_i + 1;
                    neighlabel= s(2)*s(1)*(k-1) + s(1)*(j) + i;
                    L(lindex+1,lindex_i) = neighlabel;
                     if d(i,j,k) == -1 | d(i,j,k)> 0
                        neighcount = neighcount+1;
                        neigh(count,neighcount)= neighlabel;
                     end
                  elseif d(i,j+1,k)>0
                     if d(i,j,k) == -1 | d(i,j,k)> 0
                        neighlabel= s(2)*s(1)*(k-1) + s(1)*(j) + i;
                        neighcount = neighcount+1;
                         neigh(count,neighcount)= neighlabel;
                     end
                  end
              end
              
              
              if k-1>=1 
                  if d(i,j,k-1)==-1
                    d(i,j,k-1) = d(i,j,k)+1;
                    lindex_i = lindex_i + 1;
                    neighlabel= s(2)*s(1)*(k-2) + s(1)*(j-1) + i;
                    L(lindex+1,lindex_i) = neighlabel;
                     if d(i,j,k) == -1 | d(i,j,k)> 0
                        neighcount = neighcount+1;
                         neigh(count,neighcount)= neighlabel;
                     end
                  elseif d(i,j,k-1)>0
                     if d(i,j,k) == -1 | d(i,j,k)> 0
                        neighlabel= s(2)*s(1)*(k-2) + s(1)*(j-1) + i;
                         neighcount = neighcount+1;
                         neigh(count,neighcount)= neighlabel;
                     end
                  end
              end
              
              if k+1<=s(3) 
                  if d(i,j,k+1)==-1
                    d(i,j,k+1) = d(i,j,k)+1;
                    lindex_i = lindex_i + 1;
                    neighlabel= s(2)*s(1)*(k) + s(1)*(j-1) + i;
                    L(lindex+1,lindex_i) = neighlabel;
                     if d(i,j,k) == -1 | d(i,j,k)> 0
                        neighcount = neighcount+1;
                        neigh(count,neighcount)= neighlabel;
                     end
                  elseif d(i,j,k+1)>0
                     if d(i,j,k) == -1 | d(i,j,k)> 0
                         neighlabel=  s(2)*s(1)*(k) + s(1)*(j-1) + i;
                            neighcount = neighcount+1;
                            neigh(count,neighcount)= neighlabel;
                     end
                  end
              end
              
        end
        
               
        %Update List l
        %If at least a cell has been added to the new list then proceed 
        if lindex_i >0
            l = L(lindex+1,:);
            lindex = lindex+1;
            %L
        %otherwise the algorithm has reached its end
        else
            lindex %print the maximum level of lists proceesed
            lindex=-1;%set flag to terminate the while loop
        end
     end

    
    %Print the distance-map
    fid = fopen(filename, 'wt');
    fprintf(fid, '%d %d %d\n', [s(1) s(2) s(3)]);
    for k=1:1:s(3)%pisos
      for j=1:1:s(2)%columnes
        for i=1:1:s(1)%files
          if cspace(i,j,k)==1 
            q = s(2)*s(1)*(k-1) + s(1)*(j-1) + i;
            fprintf(fid, '%d %d\n', [q d(i,j,k)]);
          end
        end
      end
    end
    fclose(fid)
        
    %Print the neighbor-map
    count=0;
    fid = fopen(filenameNeighs, 'wt');
    s=size(neigh);
    for k=1:1:s(1)
        for n=1:1:7
           if neigh(k,n)>-1
               fprintf(fid, '%d ', neigh(k,n));
           end
        end
       fprintf(fid, '-1 \n');
    end
    fclose(fid);
end
  