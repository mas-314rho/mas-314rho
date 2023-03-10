def twoSum(nums, target):
        """
        :type nums: List[int]
        :type target: int
        :rtype: List[int]
        """
        num_index = {}
    
        for i in range(len(nums)):
            complement = target - nums[i]
            if complement in num_index:
                return [num_index[complement], i]
            num_index[nums[i]] = i
        
        return None  # No solution found
    
num=[2,7,11,15]
target=9
res=twoSum(nums=num,target=target)

#Time complexity: O(n)
#Space complexity: O(n)

print(res)

