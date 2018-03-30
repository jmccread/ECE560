classdef Queue < handle
% Queue 
% Implemetation of a queue.
% Usage:
% 
% 
%
% Created by Stan Baek on 2013-05-04
% Last updated: 2013-05-04
% Stan Baek

    properties(GetAccess = 'public', SetAccess = 'private')
    % public read access, but private write access.
     
    end

    properties(GetAccess = 'private', SetAccess = 'private')
    % private read and write access
        Buffer = containers.Map('KeyType', 'int64', 'ValueType', 'any');
        FirstIndex = 1;
        LastIndex = 0;
    end
            
    methods
    
        function self = Queue()
        % class constructor
           
        end
        
        function Push(self, item)           
            
            self.LastIndex = self.LastIndex + 1;
            self.Buffer(self.LastIndex) = item;

        end
       
        function item = GetFirst(self)
            
            if (~isKey(self.Buffer, self.FirstIndex))
                disp('key does not exist.');
            end
            item = self.Buffer(self.FirstIndex);
            remove(self.Buffer, self.FirstIndex);
            self.FirstIndex = self.FirstIndex + 1;
        end
        
        function item = GetLast(self)
            item = self.Buffer(self.LastIndex);
            remove(self.Buffer, self.LastIndex);            
            self.LastIndex = self.LastIndex - 1;
        end        
        
        function size = Size(self)
            size = self.Buffer.Count;
        end
        
        function size = GetSize(self)
            size = self.Buffer.Count;
        end
        
        function bool = IsEmpty(self)
            bool = (self.Buffer.Count == 0);
        end
       
    end
    
    methods (Static, Access = public)
        
        function RunTests()
            
            q = Queue();
            
            q.Push('first');
            q.Push('second');
            q.Push('third');
            fprintf('The first element in the buffer, %s, is removed\n', q.GetFirst());
            fprintf('The last element in the buffer, %s, is removed\n',  q.GetLast());
            q.Push('fourth');
            q.Push('2');
                        
            while (q.GetSize() > 0)
                fprintf('The element in the buffer is %s\n', q.GetFirst());                
            end
                 
        end
        
        
    end
end

        